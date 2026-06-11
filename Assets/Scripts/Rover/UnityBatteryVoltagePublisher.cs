using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

public class UnityBatteryVoltagePublisher : MonoBehaviour
{
    [Header("ROS")]
    public string batteryVoltageTopic = "/battery/voltage";
    public float publishRateHz = 1f;

    [Header("Voltage")]
    public float startingVoltage = 12.6f;
    public float minimumVoltage = 10.5f;
    public float drainVoltsPerMinute = 0.02f;
    public bool simulateDrain = true;

    [Header("Debug")]
    public float currentVoltage;

    ROSConnection ros;
    string resolvedBatteryVoltageTopic;
    float nextPublishTime;

    void Start()
    {
        currentVoltage = startingVoltage;

        resolvedBatteryVoltageTopic = UnityTopicRemapper.Resolve(
            this, batteryVoltageTopic, UnityTopicRemapper.TopicKind.Sensor);

        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<Float32Msg>(resolvedBatteryVoltageTopic);
    }

    void Update()
    {
        if (simulateDrain)
        {
            float drainPerSecond = drainVoltsPerMinute / 60f;
            currentVoltage = Mathf.Max(minimumVoltage, currentVoltage - drainPerSecond * Time.deltaTime);
        }

        if (Time.time < nextPublishTime)
        {
            return;
        }

        PublishBatteryVoltage();
        nextPublishTime = Time.time + (1f / publishRateHz);
    }

    void PublishBatteryVoltage()
    {
        Float32Msg voltage = new Float32Msg
        {
            data = currentVoltage
        };

        ros.Publish(resolvedBatteryVoltageTopic, voltage);
    }
}
