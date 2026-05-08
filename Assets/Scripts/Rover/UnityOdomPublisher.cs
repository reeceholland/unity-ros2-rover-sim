using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Std;
using RosMessageTypes.Geometry;
using RosMessageTypes.Nav;
using RosMessageTypes.Tf2;

public class UnityOdomPublisher : MonoBehaviour
{
    [Header("ROS Topics")]
    public string odomTopic = "/odom";
    public string tfTopic = "/tf";

    [Header("Frames")]
    public string odomFrame = "odom";
    public string baseFrame = "base_link";

    [Header("Publishing")]
    public float publishRateHz = 30f;

    private ROSConnection ros;
    private float nextPublishTime;

    private Vector3 lastPosition;
    private Quaternion lastRotation;
    private float lastTime;


    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();

        ros.RegisterPublisher<OdometryMsg>(odomTopic);
        ros.RegisterPublisher<TFMessageMsg>(tfTopic);

        lastPosition = transform.position;
        lastRotation = transform.rotation;
        lastTime = Time.time;
    }

    void Update()
    {
        if (Time.time < nextPublishTime)
            return;

        float now = Time.time;
        float dt = Mathf.Max(now - lastTime, 0.0001f);

        PublishOdom(now, dt);

        nextPublishTime = now + (1f / publishRateHz);
        lastPosition = transform.position;
        lastRotation = transform.rotation;
        lastTime = now;
    }

    void PublishOdom(float now, float dt)
    {
        TimeMsg stamp = RosTimeUtils.Now();

        Vector3 rosPosition = UnityToRosPosition(transform.position);
        Quaternion rosRotation = UnityToRosRotation(transform.rotation);

        Vector3 unityLinearVelocity = (transform.position - lastPosition) / dt;
        Vector3 rosLinearVelocity = UnityToRosVector(unityLinearVelocity);

        Vector3 unityAngularVelocity = EstimateAngularVelocity(lastRotation, transform.rotation, dt);
        Vector3 rosAngularVelocity = UnityToRosVector(unityAngularVelocity);

        HeaderMsg header = new HeaderMsg
        {
            stamp = stamp,
            frame_id = odomFrame
        };

        PoseMsg pose = new PoseMsg
        {
            position = new PointMsg(rosPosition.x, rosPosition.y, rosPosition.z),
            orientation = new QuaternionMsg(rosRotation.x, rosRotation.y, rosRotation.z, rosRotation.w)
        };

        TwistMsg twist = new TwistMsg
        {
            linear = new Vector3Msg(rosLinearVelocity.x, rosLinearVelocity.y, rosLinearVelocity.z),
            angular = new Vector3Msg(rosAngularVelocity.x, rosAngularVelocity.y, rosAngularVelocity.z)
        };

        OdometryMsg odom = new OdometryMsg
        {
            header = header,
            child_frame_id = baseFrame,
            pose = new PoseWithCovarianceMsg
            {
                pose = pose,
                covariance = new double[36]
            },
            twist = new TwistWithCovarianceMsg
            {
                twist = twist,
                covariance = new double[36]
            }
        };

        TransformStampedMsg odomTf = new TransformStampedMsg
        {
            header = header,
            child_frame_id = baseFrame,
            transform = new TransformMsg
            {
                translation = new Vector3Msg(rosPosition.x, rosPosition.y, rosPosition.z),
                rotation = new QuaternionMsg(rosRotation.x, rosRotation.y, rosRotation.z, rosRotation.w)
            }
        };

        TFMessageMsg tfMessage = new TFMessageMsg
        {
            transforms = new TransformStampedMsg[] { odomTf }
        };

        ros.Publish(odomTopic, odom);
        ros.Publish(tfTopic, tfMessage);
    }

    TimeMsg ToRosTime(float unityTime)
    {
        int sec = Mathf.FloorToInt(unityTime);
        uint nanosec = (uint)((unityTime - sec) * 1_000_000_000f);

        return new TimeMsg(sec, nanosec);
    }

    Vector3 UnityToRosPosition(Vector3 unity)
    {
        return new Vector3(
            unity.z,
            -unity.x,
            unity.y
        );
    }

    Vector3 UnityToRosVector(Vector3 unity)
    {
        return new Vector3(
            unity.z,
            -unity.x,
            unity.y
        );
    }

    Quaternion UnityToRosRotation(Quaternion unity)
    {
        return new Quaternion(
            unity.z,
            -unity.x,
            unity.y,
            -unity.w
        );
    }

    Vector3 EstimateAngularVelocity(Quaternion previous, Quaternion current, float dt)
    {
        Quaternion delta = current * Quaternion.Inverse(previous);

        delta.ToAngleAxis(out float angleDegrees, out Vector3 axis);

        if (angleDegrees > 180f)
            angleDegrees -= 360f;

        if (float.IsNaN(axis.x))
            return Vector3.zero;

        float angleRadians = angleDegrees * Mathf.Deg2Rad;
        return axis * (angleRadians / dt);
    }
}
