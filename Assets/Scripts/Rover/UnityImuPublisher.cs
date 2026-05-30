using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;

// Publishes a ROS sensor_msgs/Imu message from the rover's Unity Rigidbody.
//
// Attach this to a child object where the IMU would physically sit. The script
// finds the rover Rigidbody above it and uses this object's transform as the
// IMU's sensor frame.
//
// Important frame convention:
//   Unity: X right, Y up, Z forward
//   ROS:   X forward, Y left, Z up
//
// The helper functions near the bottom do the Unity-to-ROS axis conversion.
public class UnityImuPublisher : MonoBehaviour
{
    [Header("ROS")]
    // Topic that robot_localization or another state estimator can consume.
    public string imuTopic = "/sensors/imu";

    // This should match the IMU link name in the URDF.
    // Example URDF:
    //   <link name="imu_link"/>
    //   <joint name="base_to_imu" ... child link="imu_link"/>
    public string frameId = "imu_link";

    // 50 Hz is a sensible starting point for a simulated rover IMU.
    public float publishRateHz = 50f;

    [Header("Published Fields")]
    // Publish orientation from this object's Unity transform.
    // If disabled, the covariance marks orientation as unavailable.
    public bool publishOrientation = true;

    // Estimate acceleration from Rigidbody velocity changes.
    // If disabled, the covariance marks acceleration as unavailable.
    public bool publishLinearAcceleration = true;

    // Shared ROS-TCP-Connector instance.
    ROSConnection ros;

    // The rover Rigidbody gives us the motion data we turn into IMU readings.
    Rigidbody roverBody;

    // Previous physics velocity, used for acceleration = delta velocity / time.
    Vector3 lastVelocity;

    // Used to publish at publishRateHz even if Unity's physics rate changes.
    float lastPublishTime;

    void Start()
    {
        // Register the ROS publisher once when the scene starts.
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<ImuMsg>(imuTopic);

        // The IMU is usually a child object, so look upward for the rover body.
        roverBody = GetComponentInParent<Rigidbody>();

        if (roverBody == null)
        {
            Debug.LogError("[UnityImuPublisher] No Rigidbody found in parent rover.");

            // Stop this component if there is no body to measure.
            enabled = false;
            return;
        }

        // Seed the previous velocity so the first acceleration sample is sane.
        lastVelocity = roverBody.linearVelocity;
    }

    void FixedUpdate()
    {
        // FixedUpdate follows the physics loop, but the ROS topic gets its own
        // rate so changing Unity's timestep will not silently change IMU output.
        if (Time.time - lastPublishTime < 1f / publishRateHz)
        {
            return;
        }

        // Clamp the timestep so the acceleration calculation cannot divide by 0.
        float dt = Mathf.Max(Time.fixedDeltaTime, 0.0001f);

        PublishImu(dt);

        // Keep the current velocity for the next delta-velocity calculation.
        lastPublishTime = Time.time;
        lastVelocity = roverBody.linearVelocity;
    }

    void PublishImu(float dt)
    {
        // Use the same sim-time helper as the rest of the Unity publishers.
        HeaderMsg header = new HeaderMsg
        {
            stamp = RosTimeUtils.Now(),
            frame_id = frameId
        };

        // -----------------------------
        // Orientation
        // -----------------------------
        // ROS stores orientation as a geometry_msgs/Quaternion.
        QuaternionMsg orientation = new QuaternionMsg();

        // Covariance is a flattened 3x3 matrix. The diagonal entries are at
        // [0], [4], and [8].
        double[] orientationCovariance = new double[9];

        if (publishOrientation)
        {
            // Convert this Unity orientation into the ROS frame convention.
            UnityEngine.Quaternion rosOrientation =
                UnityToRosRotation(transform.rotation);

            orientation = new QuaternionMsg(
                rosOrientation.x,
                rosOrientation.y,
                rosOrientation.z,
                rosOrientation.w
            );

            // These are placeholder sim covariances, not measured sensor noise.
            orientationCovariance[0] = 0.02;
            orientationCovariance[4] = 0.02;
            orientationCovariance[8] = 0.02;
        }
        else
        {
            // In sensor_msgs/Imu, -1 in the first covariance slot means
            // "this field is not provided."
            orientationCovariance[0] = -1.0;
        }

        // -----------------------------
        // Angular Velocity
        // -----------------------------
        // Unity gives angular velocity in world axes. An IMU reports it in the
        // sensor's local axes, so convert world motion into this object's frame.
        Vector3 unityLocalAngularVelocity =
            transform.InverseTransformDirection(roverBody.angularVelocity);

        // Then switch from Unity axes to ROS axes.
        Vector3 rosAngularVelocity =
            UnityToRosVector(unityLocalAngularVelocity);

        Vector3Msg angularVelocity = new Vector3Msg(
            rosAngularVelocity.x,
            rosAngularVelocity.y,
            rosAngularVelocity.z
        );

        double[] angularVelocityCovariance = new double[9];
        angularVelocityCovariance[0] = 0.01;
        angularVelocityCovariance[4] = 0.01;
        angularVelocityCovariance[8] = 0.01;

        // -----------------------------
        // Linear Acceleration
        // -----------------------------
        Vector3Msg linearAcceleration = new Vector3Msg();
        double[] linearAccelerationCovariance = new double[9];

        if (publishLinearAcceleration)
        {
            // Estimate acceleration from how the Rigidbody velocity changed.
            Vector3 worldAcceleration =
                (roverBody.linearVelocity - lastVelocity) / dt;

            // Real accelerometers feel gravity. At rest they read about +9.81
            // m/s^2 upward, so subtract Unity gravity to mimic that behavior.
            Vector3 properAcceleration =
                worldAcceleration - Physics.gravity;

            // Publish acceleration in the IMU's local frame.
            Vector3 unityLocalAcceleration =
                transform.InverseTransformDirection(properAcceleration);

            // Match the ROS body-frame convention before publishing.
            Vector3 rosAcceleration =
                UnityToRosVector(unityLocalAcceleration);

            linearAcceleration = new Vector3Msg(
                rosAcceleration.x,
                rosAcceleration.y,
                rosAcceleration.z
            );

            // Acceleration from velocity differences is a little rough, so give
            // it a looser placeholder covariance.
            linearAccelerationCovariance[0] = 0.2;
            linearAccelerationCovariance[4] = 0.2;
            linearAccelerationCovariance[8] = 0.2;
        }
        else
        {
            // Mark acceleration as unavailable.
            linearAccelerationCovariance[0] = -1.0;
        }

        // Build the final ROS message and send it across the bridge.
        ImuMsg imu = new ImuMsg(
            header,
            orientation,
            orientationCovariance,
            angularVelocity,
            angularVelocityCovariance,
            linearAcceleration,
            linearAccelerationCovariance
        );

        ros.Publish(imuTopic, imu);
    }

    // Converts a vector from Unity's body-frame convention to ROS convention.
    //
    // Unity:
    //   +X right, +Y up, +Z forward
    //
    // ROS:
    //   +X forward, +Y left, +Z up
    //
    // Mapping:
    //   ROS X = Unity Z
    //   ROS Y = -Unity X
    //   ROS Z = Unity Y
    Vector3 UnityToRosVector(Vector3 unity)
    {
        return new Vector3(
            unity.z,
            -unity.x,
            unity.y
        );
    }

    // Converts a Unity orientation into the same ROS convention used by the
    // odom publisher.
    //
    // This returns UnityEngine.Quaternion just as a convenient four-number
    // container before copying the values into QuaternionMsg.
    UnityEngine.Quaternion UnityToRosRotation(UnityEngine.Quaternion unity)
    {
        return new UnityEngine.Quaternion(
            unity.z,
            -unity.x,
            unity.y,
            -unity.w
        );
    }
}
