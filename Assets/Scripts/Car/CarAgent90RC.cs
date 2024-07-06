using System.Collections;
using System.Collections.Generic;
using UnityEngine;
//using Debug = UnityEngine.Debug;

using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using Unity.Barracuda;
using System;
using UnityEditor;

#pragma warning disable IDE0130 // Namespace does not match folder structure
namespace UnityStandardAssets.Vehicles.Car
#pragma warning restore IDE0130 // Namespace does not match folder structure
{
    public class CarAgent90RC : Agent
    {
        public Vector2 spawnRangeX = new(1, 2.3f);
        public Vector2 spawnRangeZ = new(-9f, 2f); // -17, -11

        // Private variables
        private int RayAmount;
        private float[] RayDistancesLeftTarget;
        private float[] RayDistancesRightTarget;
        private float RayDistanceFrontTarget;
        private float RayDistanceBackTarget;

        private float[] RayDistancesLeftPrevious;
        private float[] RayDistancesRightPrevious;
        private float RayDistanceFrontPrevious;
        private float RayDistanceBackPrevious;

        private RayPerceptionOutput.RayOutput[] RawRayOutputs;

        private float PreviousDistance = 0f;

        private ActionSegment<float> LastAction;
        private float[] CurrentRays;


        // GameObjects
        public GameObject targetFinal;


        // Components
        private RayPerceptionSensorComponent3D RayPerceptionSensorComponent3D;
        private Rigidbody rb;
        private CarControllerRC carControllerRC;
        private StatsRecorder statsRecorder;
        private Vector3 startPosition;



        // Use to benchmark the agent
        private readonly Queue<bool> successQueue = new();
        private readonly Queue<float> distanceToTargetLidarQueue = new();
        private const int queueSize = 100;



        public override void Initialize()
        {
            carControllerRC = GetComponent<CarControllerRC>();
            GameObject Sensor = transform.Find("Sensor").gameObject;
            RayPerceptionSensorComponent3D = Sensor.GetComponent<RayPerceptionSensorComponent3D>();
            // RayPerceptionSensorComponent3D = GetComponent<RayPerceptionSensorComponent3D>();
            rb = GetComponent<Rigidbody>();
            statsRecorder = Academy.Instance.StatsRecorder;

            startPosition = transform.localPosition;

            // Initialize the RayDistances arrays
            RayPerceptionInput RayPerceptionIn = RayPerceptionSensorComponent3D.GetRayPerceptionInput();
            RayPerceptionOutput RayPerceptionOut = RayPerceptionSensor.Perceive(RayPerceptionIn);
            RayPerceptionOutput.RayOutput[] RayOutputs = RayPerceptionOut.RayOutputs;
    
            // Length -1 because 2 Rays overlap in the end of the array.
            RayAmount = RayOutputs.Length - 1;
            RayDistancesLeftTarget = new float[(RayAmount - 2) / 2];
            RayDistancesRightTarget = new float[(RayAmount - 2) / 2];
            RayDistancesLeftPrevious = new float[(RayAmount - 2) / 2];
            RayDistancesRightPrevious = new float[(RayAmount - 2) / 2];


            Reset();   
        }

        public override void OnEpisodeBegin()
        {
            Reset();
        }

        private void EndEpisodeWithSuccess(bool wasSuccessful, float distanceToTargetLidar = -1f)
        {
            if (successQueue.Count >= queueSize) {
                successQueue.Dequeue();
                distanceToTargetLidarQueue.Dequeue();
            }

            successQueue.Enqueue(wasSuccessful);
            distanceToTargetLidarQueue.Enqueue(distanceToTargetLidar);

            float successRate = CalculateSuccessRate();
            float averageDistance = CalculateAverageDistance();


            statsRecorder.Add("ParkingSuccessRate", successRate);
            if (distanceToTargetLidar != -1f)
                statsRecorder.Add("ParkingMultiplier", averageDistance);


            EndEpisode();
        }

        private float CalculateSuccessRate()
        {
            int successCount = 0;
            foreach (bool success in successQueue) {
                if (success)
                    successCount++;
            }

            return (float)successCount / successQueue.Count * 100;
        }

        private float CalculateAverageDistance()
        {
            float totalMultiplier = 0f;
            foreach (float multiplier in distanceToTargetLidarQueue) {
                totalMultiplier += multiplier;
            }

            return totalMultiplier / distanceToTargetLidarQueue.Count;
        }

        void FixedUpdate()
        {
            // Get desicion from python by requesting the next action
            RequestDecision();
        }

        private float CalculateReward()
        {
            float reward = 0f;

            // Negative reward for each step of 10 / MaxStep
            reward -= 1000f / MaxStep;


            float distance = CalculateDistanceDotProduct();
            // float distance = CalculateDistanceLidar(ReadRayCast()) * 100f;

            if (distance < 5f && Mathf.Abs(carControllerRC.CurrentSpeed) < 0.15) {
                reward += 10000f;
                Debug.Log("Success");
                EndEpisodeWithSuccess(true, distance);
            }

            reward += (PreviousDistance - distance) * 100;
            PreviousDistance = distance;

            // if (Mathf.Abs(reward) > 1f)
            //     Debug.Log("Reward: " + reward + " Distance: " + distance);


            return reward;
        }

        private void Reset()
        {
            // Put the car in the slot space and read the lidar
            rb.velocity = Vector3.zero;
            rb.angularVelocity = Vector3.zero;
            Vector3 ReadLidarPosition = new(targetFinal.transform.localPosition.x, startPosition.y, targetFinal.transform.localPosition.z);
            rb.transform.SetLocalPositionAndRotation(ReadLidarPosition, Quaternion.Euler(0, 0, 0));

            (RayDistancesLeftTarget, RayDistancesRightTarget, RayDistanceFrontTarget, RayDistanceBackTarget) = ReadRayCast();

        
            // Reset the car
            float spawnX = UnityEngine.Random.Range(spawnRangeX.x, spawnRangeX.y);
            float spawnZ = UnityEngine.Random.Range(spawnRangeZ.x, spawnRangeZ.y);
            Vector3 spawnPosition = new(spawnX, startPosition.y, spawnZ);
            Quaternion spawnRotation = Quaternion.Euler(0, UnityEngine.Random.Range(75f, 105f), 0);

            rb.transform.SetLocalPositionAndRotation(spawnPosition, spawnRotation);
        }


        public override void CollectObservations(VectorSensor sensor)
        {
            float[] RayDistancesLeftCurrent;
            float[] RayDistancesRightCurrent;
            float RayDistanceFrontCurrent;
            float RayDistanceBackCurrent;

            (RayDistancesLeftCurrent, RayDistancesRightCurrent, RayDistanceFrontCurrent, RayDistanceBackCurrent) = ReadRayCast();

            RayPerceptionInput RayPerceptionIn = RayPerceptionSensorComponent3D.GetRayPerceptionInput();
            RayPerceptionOutput RayPerceptionOut = RayPerceptionSensor.Perceive(RayPerceptionIn);
            RawRayOutputs = RayPerceptionOut.RayOutputs;


            // Add the lidar data to the observation
            sensor.AddObservation(RayDistanceFrontCurrent);
            for (int i = 0; i < RayDistancesRightCurrent.Length; i++)
                sensor.AddObservation(RayDistancesRightCurrent[i]);
            sensor.AddObservation(RayDistanceBackCurrent);
            for (int i = 0; i < RayDistancesLeftCurrent.Length; i++)
                sensor.AddObservation(RayDistancesLeftCurrent[i]);
        }

        public override void OnActionReceived(ActionBuffers actions)
        {
            float steering = actions.ContinuousActions[0];
            float speed = actions.ContinuousActions[1];

            LastAction = actions.ContinuousActions;

            // Debug.Log("steering: " + Mathf.Round(steering*100f)/100 + " speed: " + Mathf.Round(speed*100f)/100 + " steps: " + steps);

            carControllerRC.SetSpeed(speed);
            carControllerRC.SetSteering(steering);

            float reward = CalculateReward();

            AddReward(reward);

            // Si on appuie sur la touche espace, on affiche les données du lidar
            if (Input.GetKeyDown(KeyCode.Space)) {
                
                CurrentRays = new float[RawRayOutputs.Length - 1];
                for (int i = 0; i < RawRayOutputs.Length - 1; i++)
                    CurrentRays[i] = RawRayOutputs[i].HitFraction;

                Debug.Log("Lidar(" + CurrentRays.Length + "):" + ArrayToString(CurrentRays)); 
                Debug.Log("Last action: Steering: " + LastAction[0] + "Speed: " + LastAction[1]);
            }
        }

        public override void Heuristic(in ActionBuffers actionsOut)
        {
            ActionSegment<float> continuousActionsOut = actionsOut.ContinuousActions;

            float steering = Input.GetAxis("Horizontal");
            float speed = Input.GetAxis("Vertical");

            continuousActionsOut[0] = steering;
            continuousActionsOut[1] = speed;
        }

        private (float[] RDistL, float[] RDistR, float RDistF, float RDistB) ReadRayCast()
        {
            RayPerceptionInput RayPerceptionIn = RayPerceptionSensorComponent3D.GetRayPerceptionInput();
            RayPerceptionOutput RayPerceptionOut = RayPerceptionSensor.Perceive(RayPerceptionIn);
            RayPerceptionOutput.RayOutput[] RayOutputs = RayPerceptionOut.RayOutputs;

            // Indexing is quite retarded
            // Index ordering: [0] = front, [1] = right, [2] = left, [3] = right, [4] = left,...
            // Even numbers are left, odd numbers are right
            float[] RayDistancesLeft  = new float[(RayAmount - 2) / 2];
            float[] RayDistancesRight = new float[(RayAmount - 2) / 2];
            float RayDistanceFront = RayOutputs[0].HitFraction;
            float RayDistanceBack = RayOutputs[RayAmount - 1].HitFraction;

            for (int i = 1; i < RayAmount - 1; i++)
                if (i % 2 == 0)
                    RayDistancesLeft[(i - 2) / 2] = RayOutputs[i].HitFraction;
                else
                    RayDistancesRight[(i - 1) / 2] = RayOutputs[i].HitFraction;



            // Debug.Log("Left rays:\n" + ArrayToString(RayDistancesLeft));
            // Debug.Log("Right rays:\n" + ArrayToString(RayDistancesRight));

            return (RayDistancesLeft, RayDistancesRight, RayDistanceFront, RayDistanceBack);
        }

        private string ArrayToString(float[] array) {
            string str = "[";

            for (int i = 0; i < array.Length; i++) {
                str += array[i].ToString(System.Globalization.CultureInfo.InvariantCulture);
                if (i != array.Length - 1)
                    str += ", ";
            }

            str += "]";

            return str;
        }

        float CalculateDistanceDotProduct()
        {
            // ReadRayCast() 
            float[] RayDistancesLeftCurrent;
            float[] RayDistancesRightCurrent;
            float RayDistanceFrontCurrent;
            float RayDistanceBackCurrent;

            (RayDistancesLeftCurrent, RayDistancesRightCurrent, RayDistanceFrontCurrent, RayDistanceBackCurrent) = ReadRayCast();
            // calculate the dot product between the (target - previous) and (target - current) lidar

            float distFrontCurrent = RayDistanceFrontTarget - RayDistanceFrontCurrent;
            float distBackCurrent = RayDistanceBackTarget - RayDistanceBackCurrent;
            float[] distLeftCurrent = new float[RayDistancesLeftTarget.Length];
            float[] distRightCurrent = new float[RayDistancesRightTarget.Length];
            for (int i = 0; i < RayDistancesLeftTarget.Length; i++) {
                distLeftCurrent[i] = RayDistancesLeftTarget[i] - RayDistancesLeftCurrent[i];
                distRightCurrent[i] = RayDistancesRightTarget[i] - RayDistancesRightCurrent[i];
            }

            float distFrontPrevious = RayDistanceFrontTarget - RayDistanceFrontPrevious;
            float distBackPrevious = RayDistanceBackTarget - RayDistanceBackPrevious;
            float[] distLeftPrevious = new float[RayDistancesLeftTarget.Length];
            float[] distRightPrevious = new float[RayDistancesRightTarget.Length];
            for (int i = 0; i < RayDistancesLeftTarget.Length; i++) {
                distLeftPrevious[i] = RayDistancesLeftTarget[i] - RayDistancesLeftPrevious[i];
                distRightPrevious[i] = RayDistancesRightTarget[i] - RayDistancesRightPrevious[i];
            }

            float dotProductLeft = DotProduct(distLeftCurrent, distLeftPrevious);
            float dotProductRight = DotProduct(distRightCurrent, distRightPrevious);
            float dotProductFront = distFrontCurrent * distFrontPrevious;
            float dotProductBack = distBackCurrent * distBackPrevious;

            float distance = dotProductLeft + dotProductRight + dotProductFront + dotProductBack;

            RayDistancesLeftPrevious = RayDistancesLeftCurrent;
            RayDistancesRightPrevious = RayDistancesRightCurrent;
            RayDistanceFrontPrevious = RayDistanceFrontCurrent;
            RayDistanceBackPrevious = RayDistanceBackCurrent;

            return distance;
        }

        float DotProduct(float[] a, float[] b)
        {
            float dotProduct = 0f;

            for (int i = 0; i < a.Length; i++)
            {
                dotProduct += a[i] * b[i];
            }

            return dotProduct;
        }

        void OnTriggerEnter(Collider other)
        {
            // idem que en bas
            if (other.gameObject.CompareTag("envWall"))
            {
                AddReward(-1000f);
                EndEpisodeWithSuccess(false);
                Debug.Log("Out of bounds");
            }
        }

        void OnCollisionEnter(Collision collision)
        {
            // a suprimer et a remplacer par une condition sur le lidar
            if (collision.gameObject.CompareTag("Car"))
            {
                AddReward(-500f);
                EndEpisodeWithSuccess(false);
                Debug.Log("Collision with car");
            }
        }
    }
}


