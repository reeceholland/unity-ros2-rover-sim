using System;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

public class UnityOusterOs1Publisher : MonoBehaviour
{
    [Header("ROS")]
    public string pointCloudTopic = "/ouster/points";
    public string frameId = "os1_lidar";

    [Header("Sensor Frame")]
    [Tooltip("Optional orientation reference for the ROS sensor frame; keeps visual mesh rotation separate.")]
    public Transform orientationReference;

    Quaternion SensorRotation => orientationReference != null ? orientationReference.rotation : transform.rotation;

    [Header("OS1 Scan Pattern")]
    public int verticalChannels = 32;
    public int horizontalSamples = 512;
    public float verticalFovDegrees = 45f;
    public float publishRate = 10f;

    [Header("Range")]
    public float rangeMin = 0.3f;
    public float rangeMax = 30f;
    public LayerMask hitLayers = ~0;

    [Header("Debug Visualization")]
    public bool drawDebugRays = false;
    public int debugHorizontalStride = 32;
    public int debugVerticalStride = 4;
    public Color hitRayColor = Color.cyan;
    public Color missRayColor = new Color(1f, 0f, 0f, 0.25f);
    public float debugRayDuration = 0.1f;

    const int PointStep = 16;
    const byte Float32Datatype = 7;

    ROSConnection ros;
    string resolvedPointCloudTopic;
    float lastPublishTime;
    PointFieldMsg[] fields;

    void Start()
    {
        resolvedPointCloudTopic = UnityTopicRemapper.Resolve(
            this, pointCloudTopic, UnityTopicRemapper.TopicKind.Sensor);

        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<PointCloud2Msg>(resolvedPointCloudTopic);

        fields = new[]
        {
            new PointFieldMsg("x", 0, Float32Datatype, 1),
            new PointFieldMsg("y", 4, Float32Datatype, 1),
            new PointFieldMsg("z", 8, Float32Datatype, 1),
            new PointFieldMsg("intensity", 12, Float32Datatype, 1)
        };
    }

    void Update()
    {
        if (Time.time - lastPublishTime < 1f / publishRate)
        {
            return;
        }

        lastPublishTime = Time.time;
        PublishPointCloud();
    }

    void PublishPointCloud()
    {
        int channels = Mathf.Max(1, verticalChannels);
        int samples = Mathf.Max(1, horizontalSamples);
        int pointCount = channels * samples;
        byte[] data = new byte[pointCount * PointStep];

        float verticalMin = -verticalFovDegrees * 0.5f * Mathf.Deg2Rad;
        float verticalMax = verticalFovDegrees * 0.5f * Mathf.Deg2Rad;
        float verticalIncrement = channels == 1 ? 0f : (verticalMax - verticalMin) / (channels - 1);
        float horizontalIncrement = 2f * Mathf.PI / samples;

        for (int channel = 0; channel < channels; channel++)
        {
            float verticalAngle = verticalMin + verticalIncrement * channel;
            float verticalCos = Mathf.Cos(verticalAngle);
            float verticalSin = Mathf.Sin(verticalAngle);

            for (int sample = 0; sample < samples; sample++)
            {
                float horizontalAngle = -Mathf.PI + horizontalIncrement * sample;
                Vector3 direction = MakeUnityRayDirection(horizontalAngle, verticalCos, verticalSin);

                int pointIndex = channel * samples + sample;
                int byteOffset = pointIndex * PointStep;

                if (Physics.Raycast(transform.position, direction, out RaycastHit hit, rangeMax, hitLayers) &&
                    hit.distance >= rangeMin)
                {
                    Vector3 rosPoint = UnityPointToRosPoint(hit.point);
                    WritePoint(data, byteOffset, rosPoint.x, rosPoint.y, rosPoint.z, 1f);
                    DrawDebugRay(channel, sample, direction, hit.distance, true);
                }
                else
                {
                    WritePoint(data, byteOffset, float.NaN, float.NaN, float.NaN, 0f);
                    DrawDebugRay(channel, sample, direction, rangeMax, false);
                }
            }
        }

        PointCloud2Msg cloud = new PointCloud2Msg(
            MakeHeader(),
            (uint)channels,
            (uint)samples,
            fields,
            false,
            PointStep,
            (uint)(samples * PointStep),
            data,
            false);

        ros.Publish(resolvedPointCloudTopic, cloud);
    }

    Vector3 MakeUnityRayDirection(float horizontalAngle, float verticalCos, float verticalSin)
    {
        return SensorRotation * new Vector3(
            -verticalCos * Mathf.Sin(horizontalAngle),
            verticalSin,
            verticalCos * Mathf.Cos(horizontalAngle));
    }

    Vector3 UnityPointToRosPoint(Vector3 worldPoint)
    {
        // Convert orientation and origin only: imported mesh scale must not change metres.
        Vector3 localPoint = Quaternion.Inverse(SensorRotation) * (worldPoint - transform.position);

        return new Vector3(
            localPoint.z,
            -localPoint.x,
            localPoint.y);
    }

    void WritePoint(byte[] data, int byteOffset, float x, float y, float z, float intensity)
    {
        WriteFloat(data, byteOffset, x);
        WriteFloat(data, byteOffset + 4, y);
        WriteFloat(data, byteOffset + 8, z);
        WriteFloat(data, byteOffset + 12, intensity);
    }

    void WriteFloat(byte[] data, int byteOffset, float value)
    {
        byte[] bytes = BitConverter.GetBytes(value);
        Buffer.BlockCopy(bytes, 0, data, byteOffset, bytes.Length);
    }

    HeaderMsg MakeHeader()
    {
        HeaderMsg header = new HeaderMsg();
        header.stamp = RosTimeUtils.Now();
        header.frame_id = frameId;
        return header;
    }

    void DrawDebugRay(int channel, int sample, Vector3 direction, float distance, bool hit)
    {
        if (!drawDebugRays)
        {
            return;
        }

        if (channel % Mathf.Max(1, debugVerticalStride) != 0 ||
            sample % Mathf.Max(1, debugHorizontalStride) != 0)
        {
            return;
        }

        Debug.DrawRay(
            transform.position,
            direction * distance,
            hit ? hitRayColor : missRayColor,
            debugRayDuration);
    }
}
