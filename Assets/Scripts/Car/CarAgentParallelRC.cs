using System.Collections;
using System.Collections.Generic;
using UnityEngine;
//using Debug = UnityEngine.Debug;

using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using Unity.Barracuda;
using System;

#pragma warning disable IDE0130 // Namespace does not match folder structure
namespace UnityStandardAssets.Vehicles.Car
#pragma warning restore IDE0130 // Namespace does not match folder structure
{
    public class CarAgentParallelRC : Agent
    {
        public Vector2 spawnRangeX = new(1, 2.3f);
        public Vector2 spawnRangeZ = new(-9f, 2f); // -17, -11

        // Private variables
        private int RayAmount;
        private float[] RayDistancesLeftTarget;
        private float[] RayDistancesRightTarget;
        private float RayDistanceFrontTarget;
        private float RayDistanceBackTarget;

        // Constants
        private const float carLength = 0.042f * 100f;


        // GameObjects
        public GameObject targetFinal;
        public GameObject carBlackFront;
        public GameObject carBlackBack;


        // Components
        private RayPerceptionSensorComponent3D RayPerceptionSensorComponent3D;
        private Rigidbody rb;
        private CarControllerRC carControllerRC;
        private StatsRecorder statsRecorder;
        private Vector3 startPosition;



        // Use to benchmark the agent
        private readonly Queue<bool> successQueue = new();
        private readonly Queue<float> multiplierQueue = new();
        private const int queueSize = 100;



        public override void Initialize()
        {
            carControllerRC = GetComponent<CarControllerRC>();
            RayPerceptionSensorComponent3D = GetComponent<RayPerceptionSensorComponent3D>();
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


            Reset();   
        }

        public override void OnEpisodeBegin()
        {
            Reset();
        }

        private void EndEpisodeWithSuccess(bool wasSuccessful, float multiplier = 0f)
        {
            if (successQueue.Count >= queueSize) {
                successQueue.Dequeue();
                multiplierQueue.Dequeue();
            }

            successQueue.Enqueue(wasSuccessful);
            multiplierQueue.Enqueue(multiplier);

            float successRate = CalculateSuccessRate();
            float averageMultiplier = CalculateAverageMultiplier();


            statsRecorder.Add("ParkingSuccessRate", successRate);
            statsRecorder.Add("ParkingMultiplier", averageMultiplier);

            // Debug.Log("Reward: " + GetCumulativeReward());

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

        private float CalculateAverageMultiplier()
        {
            float totalMultiplier = 0f;
            foreach (float multiplier in multiplierQueue) {
                totalMultiplier += multiplier;
            }

            return totalMultiplier / multiplierQueue.Count;
        }

        void FixedUpdate()
        {
            // Get desicion from python by requesting the next action
            RequestDecision();
        }

        private float CalculateReward()
        {
            float reward = 0f;

            // Negative reward for each step of 100 / MaxStep
            // reward -= 100f / MaxStep;


            float distance = CalculateDistanceLidar(ReadRayCast());

            if (distance < 0.08f && Mathf.Abs(carControllerRC.CurrentSpeed) < 0.2) {
                reward += 1000f;
                Debug.Log("Success");
                EndEpisodeWithSuccess(true, distance);
            }

            reward += 1f / distance * Mathf.Abs(carControllerRC.CurrentSpeed) / 4f;

            // Debug.Log("Distance: " + distance + " Reward: " + reward);

            return reward;
        }

        private void Reset()
        {
            // Randomize the slot space
            float slotMultiplier = UnityEngine.Random.Range(1.6f, 2f); // 1 is the car length, 2 is 2 car lengths, etc.
            float currentSlotSpace = carBlackFront.transform.localPosition.z - carBlackBack.transform.localPosition.z - carLength;
            float newSlotSpace = slotMultiplier * carLength;
            float slotSpaceDiff = newSlotSpace - currentSlotSpace;
            
            carBlackFront.transform.localPosition = new Vector3(carBlackFront.transform.localPosition.x, carBlackFront.transform.localPosition.y, carBlackFront.transform.localPosition.z + slotSpaceDiff / 2);
            carBlackBack.transform.localPosition = new Vector3(carBlackBack.transform.localPosition.x, carBlackBack.transform.localPosition.y, carBlackBack.transform.localPosition.z - slotSpaceDiff / 2);
            

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
            Quaternion spawnRotation = Quaternion.Euler(0, UnityEngine.Random.Range(-5f, 5f), 0);

            rb.transform.SetLocalPositionAndRotation(spawnPosition, spawnRotation);
        }


        public override void CollectObservations(VectorSensor sensor)
        {
        }

        public override void OnActionReceived(ActionBuffers actions)
        {
            float steering = actions.ContinuousActions[0];
            float speed = actions.ContinuousActions[1];

            // Debug.Log("steering: " + Mathf.Round(steering*100f)/100 + " speed: " + Mathf.Round(speed*100f)/100 + " steps: " + steps);

            carControllerRC.SetSpeed(speed);
            carControllerRC.SetSteering(steering);

            float reward = CalculateReward();

            AddReward(reward);
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

        private string ArrayToString(float[] array){
            string str = "[";

            for(int i = 0; i < array.Length; i++){
                str += array[i] + "  ";
            }

            str += "]";

            return str;
        }

        float CalculateDistanceLidar((float[] RayDistancesLeft, float[] RayDistancesRight, float RayDistanceFront, float RayDistanceBack) rayData) 
        {
            var (RayDistancesLeft, RayDistancesRight, RayDistanceFront, RayDistanceBack) = rayData;
            float distance = 0f;

            // Calculate the distance between the targetDistance and the current distance lidar
            for (int i = 0; i < RayDistancesLeft.Length; i++)
            {
                distance += Mathf.Abs(RayDistancesLeft[i] - RayDistancesLeftTarget[i]);
                distance += Mathf.Abs(RayDistancesRight[i] - RayDistancesRightTarget[i]);
            }

            distance += Mathf.Abs(RayDistanceFront - RayDistanceFrontTarget);
            distance += Mathf.Abs(RayDistanceBack - RayDistanceBackTarget);

            distance /= (RayDistancesLeft.Length * 2 + 2);

            return distance;
        }

        void OnTriggerEnter(Collider other)
        {
            // idem que en bas
            if (other.gameObject.CompareTag("envWall"))
            {
                AddReward(-100f);
                EndEpisodeWithSuccess(false);
                Debug.Log("Out of bounds");
            }
        }

        void OnCollisionEnter(Collision collision)
        {
            // a suprimer et a remplacer par une condition sur le lidar
            if (collision.gameObject.CompareTag("Car"))
            {
                AddReward(-100f);
                EndEpisodeWithSuccess(false);
                Debug.Log("Collision with car");
            }
        }
    }
}


