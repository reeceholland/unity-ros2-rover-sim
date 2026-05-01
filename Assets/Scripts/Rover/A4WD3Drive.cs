using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

public class A4WD3Drive : MonoBehaviour
{
    const float RpmToRadPerSecond = Mathf.PI * 2f / 60f;

    [Header("Wheel Colliders")]
    public WheelCollider frontLeft;
    public WheelCollider frontRight;
    public WheelCollider rearLeft;
    public WheelCollider rearRight;

    [Header("Visual Wheels")]
    public Transform frontLeftMesh;
    public Transform frontRightMesh;
    public Transform rearLeftMesh;
    public Transform rearRightMesh;

    [Header("Drive")]
    public float motorTorque = 2f;
    public float turnTorque = 1.5f;
    public float brakeTorque = 5f;

    [Header("ROS")]
    public bool useRosCommands = true;
    public string commandTopic = "/platform/motors/cmd";
    public string feedbackTopic = "/platform/motors/feedback";
    public float commandTimeout = 0.5f;
    public float feedbackPublishRate = 20f;
    public float stoppedVelocityThreshold = 0.05f;

    [Header("Wheel Velocity PID")]
    public WheelPidController frontLeftPid = new WheelPidController();
    public WheelPidController frontRightPid = new WheelPidController();
    public WheelPidController rearLeftPid = new WheelPidController();
    public WheelPidController rearRightPid = new WheelPidController();

    Quaternion frontLeftOffset;
    Quaternion frontRightOffset;
    Quaternion rearLeftOffset;
    Quaternion rearRightOffset;

    ROSConnection ros;

    // These names must match the ROS 2 controller config and URDF joint names.
    readonly string[] jointNames =
    {
        "front_left_joint",
        "front_right_joint",
        "rear_left_joint",
        "rear_right_joint"
    };
    readonly float[] targetVelocities = new float[4];
    readonly double[] jointPositions = new double[4];
    float lastCommandTime = -1f;
    float lastFeedbackPublishTime;
    bool wasUsingRosCommand;

    void Start()
    {
        // Store the original mesh/collider rotation difference so visual wheels stay aligned.
        frontLeftOffset = GetWheelOffset(frontLeft, frontLeftMesh);
        frontRightOffset = GetWheelOffset(frontRight, frontRightMesh);
        rearLeftOffset = GetWheelOffset(rearLeft, rearLeftMesh);
        rearRightOffset = GetWheelOffset(rearRight, rearRightMesh);

        // Unity acts as the simulated hardware behind ros2_control.
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<JointStateMsg>(commandTopic, OnMotorCommand);
        ros.RegisterPublisher<JointStateMsg>(feedbackTopic);
    }

    void FixedUpdate()
    {
        // ROS velocity commands take priority while fresh; keyboard input is a local fallback.
        bool hasFreshRosCommand = useRosCommands && lastCommandTime >= 0f &&
            Time.time - lastCommandTime <= commandTimeout;

        if (hasFreshRosCommand)
        {
            ApplyRosVelocityControl();
        }
        else
        {
            if (wasUsingRosCommand)
            {
                ResetPidControllers();
            }

            ApplyKeyboardControl();
        }

        wasUsingRosCommand = hasFreshRosCommand;

        UpdateJointPositions();
        PublishFeedback();

        UpdateWheelVisual(frontLeft, frontLeftMesh, frontLeftOffset);
        UpdateWheelVisual(frontRight, frontRightMesh, frontRightOffset);
        UpdateWheelVisual(rearLeft, rearLeftMesh, rearLeftOffset);
        UpdateWheelVisual(rearRight, rearRightMesh, rearRightOffset);
    }

    void OnMotorCommand(JointStateMsg msg)
    {
        // JointState arrays are matched by name, so message order does not matter.
        for (int i = 0; i < msg.name.Length && i < msg.velocity.Length; i++)
        {
            int jointIndex = System.Array.IndexOf(jointNames, msg.name[i]);
            if (jointIndex >= 0)
            {
                targetVelocities[jointIndex] = (float)msg.velocity[i];
            }
        }

        lastCommandTime = Time.time;
    }

    void ApplyRosVelocityControl()
    {
        // Near-zero commands should hold still, not hunt around zero velocity.
        bool stoppedCommand =
            Mathf.Abs(targetVelocities[0]) < stoppedVelocityThreshold &&
            Mathf.Abs(targetVelocities[1]) < stoppedVelocityThreshold &&
            Mathf.Abs(targetVelocities[2]) < stoppedVelocityThreshold &&
            Mathf.Abs(targetVelocities[3]) < stoppedVelocityThreshold;

        if (stoppedCommand)
        {
            ResetPidControllers();
            ApplyWheel(frontLeft, 0f);
            ApplyWheel(frontRight, 0f);
            ApplyWheel(rearLeft, 0f);
            ApplyWheel(rearRight, 0f);
            SetBrake(brakeTorque);
            return;
        }

        ApplyVelocityTarget(frontLeft, frontLeftPid, targetVelocities[0]);
        ApplyVelocityTarget(frontRight, frontRightPid, targetVelocities[1]);
        ApplyVelocityTarget(rearLeft, rearLeftPid, targetVelocities[2]);
        ApplyVelocityTarget(rearRight, rearRightPid, targetVelocities[3]);
        SetBrake(0f);
    }

    void ApplyVelocityTarget(WheelCollider wheel, WheelPidController pid, float targetRadPerSecond)
    {
        // ROS commands are rad/s; Unity WheelCollider feedback is rpm.
        float currentRadPerSecond = wheel.rpm * RpmToRadPerSecond;
        wheel.motorTorque = pid.Update(targetRadPerSecond, currentRadPerSecond, Time.fixedDeltaTime);
    }

    void ApplyKeyboardControl()
    {
        float forward = 0f;
        float turn = 0f;

        if (Input.GetKey(KeyCode.W)) forward += 1f;
        if (Input.GetKey(KeyCode.S)) forward -= 1f;
        if (Input.GetKey(KeyCode.D)) turn += 1f;
        if (Input.GetKey(KeyCode.A)) turn -= 1f;

        float leftTorque = (forward * motorTorque) - (turn * turnTorque);
        float rightTorque = (forward * motorTorque) + (turn * turnTorque);

        ApplyWheel(frontLeft, leftTorque);
        ApplyWheel(rearLeft, leftTorque);
        ApplyWheel(frontRight, rightTorque);
        ApplyWheel(rearRight, rightTorque);

        bool braking = Mathf.Approximately(forward, 0f) && Mathf.Approximately(turn, 0f);
        SetBrake(braking ? brakeTorque : 0f);
    }

    Quaternion GetWheelOffset(WheelCollider collider, Transform mesh)
    {
        collider.GetWorldPose(out _, out Quaternion colliderRotation);
        return Quaternion.Inverse(colliderRotation) * mesh.rotation;
    }

    void ApplyWheel(WheelCollider wheel, float torque)
    {
        wheel.motorTorque = torque;
    }

    void SetBrake(float torque)
    {
        frontLeft.brakeTorque = torque;
        frontRight.brakeTorque = torque;
        rearLeft.brakeTorque = torque;
        rearRight.brakeTorque = torque;
    }

    void UpdateWheelVisual(WheelCollider collider, Transform mesh, Quaternion offset)
    {
        collider.GetWorldPose(out Vector3 position, out Quaternion rotation);
        mesh.position = position;
        mesh.rotation = rotation * offset;
    }

    void ResetPidControllers()
    {
        // Drop stored integral/derivative state when changing control modes or stopping.
        frontLeftPid.Reset();
        frontRightPid.Reset();
        rearLeftPid.Reset();
        rearRightPid.Reset();
    }

    void UpdateJointPositions()
    {
        // Integrate wheel velocity so ros2_control receives both position and velocity feedback.
        float dt = Time.fixedDeltaTime;
        jointPositions[0] += frontLeft.rpm * RpmToRadPerSecond * dt;
        jointPositions[1] += frontRight.rpm * RpmToRadPerSecond * dt;
        jointPositions[2] += rearLeft.rpm * RpmToRadPerSecond * dt;
        jointPositions[3] += rearRight.rpm * RpmToRadPerSecond * dt;
    }

    void PublishFeedback()
    {
        if (ros == null || Time.time - lastFeedbackPublishTime < 1f / feedbackPublishRate)
        {
            return;
        }

        lastFeedbackPublishTime = Time.time;

        ros.Publish(feedbackTopic, new JointStateMsg(
            MakeHeader(),
            jointNames,
            jointPositions,
            new double[]
            {
                frontLeft.rpm * RpmToRadPerSecond,
                frontRight.rpm * RpmToRadPerSecond,
                rearLeft.rpm * RpmToRadPerSecond,
                rearRight.rpm * RpmToRadPerSecond
            },
            new double[0]));
    }

    HeaderMsg MakeHeader()
    {
        // Use Unity realtime as a simple monotonic stamp for simulated feedback messages.
        double now = Time.realtimeSinceStartupAsDouble;
        int seconds = (int)now;
        uint nanoseconds = (uint)((now - seconds) * 1e9);

        TimeMsg stamp = new TimeMsg();
#if ROS2
        stamp.sec = seconds;
#else
        stamp.sec = (uint)seconds;
#endif
        stamp.nanosec = nanoseconds;

        HeaderMsg header = new HeaderMsg();
        header.stamp = stamp;
        header.frame_id = "base_link";
        return header;
    }
}
