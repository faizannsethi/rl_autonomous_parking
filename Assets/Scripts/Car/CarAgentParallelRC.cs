using System.Collections;
using System.Collections.Generic;
using UnityEngine;
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
    public class CarAgentParallelRC : Agent
    {
        // Public variables
        public Vector2 spawnRangeX = new(1, 2.3f);
        public Vector2 spawnRangeZ = new(-9f, 2f); // -17, -11
        private float slotMultiplier; // 1 is the car length, 2 is 2 car lengths, etc.
        public GameObject targetIntermediate;
        public GameObject targetFinal;
        public GameObject carBlackFront;
        public GameObject carBlackBack;

        // Private variables
        private int RayAmount;
        private float[] LidarMinus1;
        private float[] LidarMinus2;
        private Vector3 startPosition;
        private Vector3 lastPosition;
        private float lastAngle;
        private const float carLength = 0.042f * 100f;
        private bool inIntermediateTarget = false;
        private bool inFinishTarget = false;

        // Components
        private RayPerceptionSensorComponent3D RayPerceptionSensorComponent3D;
        private Rigidbody rb;
        private CarControllerRC carControllerRC;
        private GameObject currentTarget;
        private EnvironmentParameters defaultParameters;
        private StatsRecorder statsRecorder;

        // Queues for benchmarking the agent
        private readonly Queue<bool> successQueue = new();
        private readonly Queue<float> multiplierQueue = new();
        private const int queueSize = 100;


        public override void Initialize()
        {
            GameObject Sensor = transform.Find("Sensor").gameObject;
            RayPerceptionSensorComponent3D = Sensor.GetComponent<RayPerceptionSensorComponent3D>();
            carControllerRC = GetComponent<CarControllerRC>();
            rb = GetComponent<Rigidbody>();
            defaultParameters = Academy.Instance.EnvironmentParameters;
            startPosition = transform.localPosition;
            lastPosition = startPosition;
            statsRecorder = Academy.Instance.StatsRecorder;

            // Initialize the RayDistances arrays
            RayPerceptionInput RayPerceptionIn = RayPerceptionSensorComponent3D.GetRayPerceptionInput();
            RayPerceptionOutput RayPerceptionOut = RayPerceptionSensor.Perceive(RayPerceptionIn);
            RayPerceptionOutput.RayOutput[] RayOutputs = RayPerceptionOut.RayOutputs;
    
            // 'Length -1' because 2 Rays overlap in the end of the array.
            RayAmount = RayOutputs.Length - 1;
            LidarMinus1 = new float[RayAmount];
            LidarMinus2 = new float[RayAmount];

            Reset();
        }

        public override void OnEpisodeBegin()
        {
            Reset();
        }

        private void Reset()
        {
            // Randomize car's spawn position, rotation and slot space
            float spawnX = UnityEngine.Random.Range(spawnRangeX.x, spawnRangeX.y);
            float spawnZ = UnityEngine.Random.Range(spawnRangeZ.x, spawnRangeZ.y);
            slotMultiplier = UnityEngine.Random.Range(1.8f, 2f); // 1 is the car length, 2 is 2 car lengths, etc.
            Vector3 spawnPosition = new(spawnX, startPosition.y, spawnZ);
            Quaternion spawnRotation = Quaternion.Euler(0, UnityEngine.Random.Range(-5f, 5f), 0);

            // Adjust the slot space
            float currentSlotSpace = carBlackFront.transform.localPosition.z - carBlackBack.transform.localPosition.z - carLength;
            float newSlotSpace = slotMultiplier * carLength;
            float slotSpaceDiff = newSlotSpace - currentSlotSpace;
            carBlackFront.transform.localPosition = new Vector3(carBlackFront.transform.localPosition.x, carBlackFront.transform.localPosition.y, carBlackFront.transform.localPosition.z + slotSpaceDiff / 2);
            carBlackBack.transform.localPosition = new Vector3(carBlackBack.transform.localPosition.x, carBlackBack.transform.localPosition.y, carBlackBack.transform.localPosition.z - slotSpaceDiff / 2);
            targetIntermediate.transform.localPosition = new Vector3(targetIntermediate.transform.localPosition.x, targetIntermediate.transform.localPosition.y, carBlackFront.transform.localPosition.z - 0.8f);

            // Set the car's position and rotation
            rb.transform.SetLocalPositionAndRotation(spawnPosition, spawnRotation);
            rb.velocity = Vector3.zero;
            rb.angularVelocity = Vector3.zero;

            // Reset variables
            inIntermediateTarget = false;
            inFinishTarget = false;

            currentTarget = targetIntermediate;
            targetIntermediate.SetActive(true);
            targetFinal.SetActive(false);
        }

        public override void CollectObservations(VectorSensor sensor)
        {
            // Collect observations from the environment
            float[] CurrentLidar = ReadCurrentLidar();

            // Add the observations to the sensor
            for (int i = 0; i < CurrentLidar.Length; i++) {
                sensor.AddObservation(CurrentLidar[i]);
                sensor.AddObservation(LidarMinus1[i]);
                sensor.AddObservation(LidarMinus2[i]);
            }

            sensor.AddObservation(carControllerRC.CurrentSpeed / carControllerRC.MaxSpeed);
            
            LidarMinus2 = LidarMinus1;
            LidarMinus1 = CurrentLidar;
        }

        public override void OnActionReceived(ActionBuffers actions)
        {
            float steering = actions.ContinuousActions[0];
            float speed = actions.ContinuousActions[1];

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

        void FixedUpdate()
        {
            RequestDecision();
        }

        private float CalculateReward()
        {
            float reward = 0f;

            // Negative reward for each step of 100 / MaxStep
            reward -= 100f / MaxStep;

            // If the agent is close enough to the intermediate target, switch to the final target
            if (inIntermediateTarget)
            {
                float distanceToTargetX = Mathf.Abs(transform.localPosition.x - targetIntermediate.transform.localPosition.x);
                float distanceToTargetZ = Mathf.Abs(transform.localPosition.z - targetIntermediate.transform.localPosition.z);

                if (distanceToTargetX < 1.5f && distanceToTargetZ < 1.5f)
                {
                    currentTarget = targetFinal;
                    inIntermediateTarget = false;

                    targetIntermediate.SetActive(false);
                    targetFinal.SetActive(true);

                    reward += 100f;

                    Debug.Log("Switched to final target");
                }
            }

            // Reward for aligning with the target and being aligned with it
            if (inFinishTarget && lastAngle != 0f)
            {
                float angleToTarget = Vector3.Angle(transform.forward, currentTarget.transform.forward);
                float angleChangeReward = lastAngle - angleToTarget;
                angleChangeReward = Mathf.Clamp(angleChangeReward, -1f, 0.75f);

                reward += 5f * angleChangeReward * Mathf.Min(4f / angleToTarget, 1f);

                // if (Mathf.Abs(reward) > 0.08f)
                //     Debug.Log("Reward angle: " + reward);
                // else
                //     Debug.Log("Reward angle: 0");


                // Reward the agent if it's parked
                float distanceToTargetX = Mathf.Abs(transform.localPosition.x - currentTarget.transform.localPosition.x);
                if (angleToTarget < 3f && distanceToTargetX < 1f && Mathf.Abs(carControllerRC.CurrentSpeed) < 0.05f)
                {
                    float multiplier = 1f - distanceToTargetX;
                    // reward += -2000 + 10000f * multiplier * multiplier;

                    if (multiplier < 0.6f)
                    {
                        reward += 10000f * multiplier;
                    }
                    else
                    {
                        reward += 6000f;
                    }

                    AddReward(reward);

                    static void CalculateGrade(int numTiers, float grade)
                    {
                        float tierSize = 1f / numTiers;
                        int tier = Mathf.FloorToInt(grade / tierSize);
                        Debug.Log("Car parked!: " + tier + "/" + numTiers);
                    }

                    CalculateGrade(10, multiplier);

                    EndEpisodeWithSuccess(true, multiplier);
                }
            }
            
            if (currentTarget == targetFinal && Mathf.Abs(transform.localPosition.x - currentTarget.transform.localPosition.x) > 2f)
            {
                // Reward for entering the finish target backwards at 45°
                float angleToTarget = Vector3.SignedAngle(transform.forward, currentTarget.transform.forward, Vector3.up);
                angleToTarget = Mathf.Clamp(angleToTarget, -10f, 45f);


                float angleToTargetReward = -carControllerRC.CurrentSpeed * angleToTarget / 45f;

                reward += angleToTargetReward;

                // Debug.Log("Angle to target reward: " + angleToTargetReward);
            }

            // Reward for moving towards the target and being close to it
            if (lastPosition != Vector3.zero)
            {
                float distanceToTargetX = Mathf.Abs(transform.localPosition.x - currentTarget.transform.localPosition.x);
                float distanceToTargetZ = Mathf.Abs(transform.localPosition.z - currentTarget.transform.localPosition.z);

                float lastDistanceToTargetX = Mathf.Abs(lastPosition.x - currentTarget.transform.localPosition.x);
                float lastDistanceToTargetZ = Mathf.Abs(lastPosition.z - currentTarget.transform.localPosition.z);

                float directionChangeX = lastDistanceToTargetX - distanceToTargetX;
                float directionChangeZ = lastDistanceToTargetZ - distanceToTargetZ;

                float totDirectionChangeReward = directionChangeX * 20f + directionChangeZ * 10f;
                totDirectionChangeReward = Mathf.Clamp(totDirectionChangeReward, -1f, 2f);

                reward += 2 * totDirectionChangeReward;

                // if (Mathf.Abs(reward) > 0.08f)
                //     Debug.Log("Reward distance: " + reward);
            }


            lastPosition = transform.localPosition;
            lastAngle = Vector3.Angle(transform.forward, currentTarget.transform.forward);

            // Debug.Log("Reward: " + reward);


            return reward;
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

        private float[] ReadCurrentLidar()
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

            float[] Lidar = new float[RayAmount];
            int index = 0;

            Lidar[index++] = RayDistanceFront;
            for (int i = 0; i < RayDistancesRight.Length; i++)
                Lidar[index++] = RayDistancesRight[i];

            Lidar[index++] = RayDistanceBack;
            for (int i = 0; i < RayDistancesLeft.Length; i++) // needs to be fixed
                Lidar[index++] = RayDistancesLeft[i];

            return Lidar;
        }

        void OnTriggerEnter(Collider other)
        {
            if (other.gameObject.CompareTag("Finish") && !inFinishTarget)
            {
                inFinishTarget = true;
                AddReward(100f);
                Debug.Log("In finish target");
            }
            else if (other.gameObject.CompareTag("IntermediateFinish") && !inIntermediateTarget)
            {
                inIntermediateTarget = true;
                Debug.Log("Intermediate target reached");
            }
            else if (other.gameObject.CompareTag("envWall"))
            {
                AddReward(-100f);
                EndEpisodeWithSuccess(false);
                Debug.Log("Out of bounds");
            }
        }

        void OnTriggerExit(Collider other)
        {
            if (other.gameObject.CompareTag("Finish") && inFinishTarget)
            {
                inFinishTarget = false;
                AddReward(-100f);
                Debug.Log("Out finish target");
            }
        }

        void OnCollisionEnter(Collision collision)
        {
            if (collision.gameObject.CompareTag("Car"))
            {
                AddReward(-100f);
                EndEpisodeWithSuccess(false);
                Debug.Log("Collision with car");
            }
        }
    }
}


