using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

public class UnityLidarPublisher : MonoBehaviour
{
    public string scanTopic = "/scan";
    public string frameId = "laser";

    public float rangeMin = 0.12f;
    public float rangeMax = 8f;
    public float fieldOfViewFDegrees = 270f;
    public int sampleCount = 541;
    public float publishRate = 10f;
    public LayerMask hitLayers = ~0; // Default to everything

    [Header("Debug Visualization")]
    public bool drawDebugRays = true;
    public bool showHitMarkers = true;
    public Color hitRayColor = Color.green;
    public Color missRayColor = new Color(1f, 0f, 0f, 0.35f);
    public Color hitMarkerColor = Color.cyan;
    public float debugRayDuration = 0.1f;
    public float hitMarkerSize = 0.035f;

    ROSConnection ros;
    float lastPublishTime;
    string resolvedScanTopic;
    Transform[] hitMarkers;
    Material hitMarkerMaterial;

    void Start()
    {
        resolvedScanTopic = UnityTopicRemapper.Resolve(
            this, scanTopic, UnityTopicRemapper.TopicKind.Sensor);

        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<LaserScanMsg>(resolvedScanTopic);
        CreateHitMarkers();
    }

    void Update()
    {
        if (Time.time - lastPublishTime < 1f / publishRate)
        {
            return;
        }

        lastPublishTime = Time.time;
        PublishScan();
    }

    void PublishScan()
    {
        float angleMin = -fieldOfViewFDegrees * 0.5f * Mathf.Deg2Rad;
        float angleMax = fieldOfViewFDegrees * 0.5f * Mathf.Deg2Rad;
        float angleIncrement = (angleMax - angleMin) / (sampleCount - 1);

        float[] ranges = new float[sampleCount];
        float[] intensities = new float[sampleCount];

        for (int i = 0; i < sampleCount; i++)
        {
            float angle = angleMin + angleIncrement * i;
            // ROS LaserScan positive angles rotate toward +Y left; Unity right is ROS -Y.
            UnityEngine.Vector3 direction = transform.forward * Mathf.Cos(angle) - transform.right * Mathf.Sin(angle);
            if (Physics.Raycast(transform.position, direction, out RaycastHit hit, rangeMax, hitLayers))
            {
                ranges[i] = hit.distance < rangeMin ? float.PositiveInfinity : hit.distance;
                intensities[i] = 1f; // Placeholder intensity value
                VisualizeRay(i, direction, hit.distance, true, hit.point);
            }
            else
            {
                ranges[i] = rangeMax;
                intensities[i] = 0f;
                VisualizeRay(i, direction, rangeMax, false, transform.position + direction * rangeMax);
            }
        }

        ros.Publish(resolvedScanTopic, new LaserScanMsg(
            MakeHeader(),
            angleMin,
            angleMax,
            angleIncrement,
            0f,
            1f / publishRate,
            rangeMin,
            rangeMax,
            ranges,
            intensities));
    }

    HeaderMsg MakeHeader()
    {
        TimeMsg stamp = RosTimeUtils.Now();

        HeaderMsg header = new HeaderMsg();
        header.stamp = stamp;
        header.frame_id = frameId;
        return header;
    }

    void VisualizeRay(int index, Vector3 direction, float distance, bool hit, Vector3 point)
    {
        if (drawDebugRays)
        {
            Debug.DrawRay(transform.position, direction * distance, hit ? hitRayColor : missRayColor, debugRayDuration);
        }

        if (!showHitMarkers || hitMarkers == null || index >= hitMarkers.Length)
        {
            return;
        }

        Transform marker = hitMarkers[index];
        marker.gameObject.SetActive(hit);
        if (hit)
        {
            marker.position = point;
        }
    }

    void CreateHitMarkers()
    {
        if (!showHitMarkers || sampleCount <= 0)
        {
            return;
        }

        hitMarkerMaterial = new Material(Shader.Find("Universal Render Pipeline/Lit"));
        hitMarkerMaterial.color = hitMarkerColor;

        hitMarkers = new Transform[sampleCount];
        for (int i = 0; i < sampleCount; i++)
        {
            GameObject marker = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            marker.name = $"LidarHit_{i:000}";
            marker.transform.SetParent(transform, true);
            marker.transform.localScale = Vector3.one * hitMarkerSize;
            marker.GetComponent<Renderer>().material = hitMarkerMaterial;
            Destroy(marker.GetComponent<Collider>());
            marker.SetActive(false);
            hitMarkers[i] = marker.transform;
        }
    }
}
