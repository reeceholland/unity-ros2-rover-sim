using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Rosgraph;

public class UnityClockPublisher : MonoBehaviour
{
    public string clockTopic = "/clock";
    public float publishRateHz = 60f;

    private ROSConnection ros;
    private float nextPublishTime;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<ClockMsg>(clockTopic);
    }

    void Update()
    {
        if (Time.time < nextPublishTime)
            return;

        ClockMsg clock = new ClockMsg
        {
            clock = RosTimeUtils.Now()
        };

        ros.Publish(clockTopic, clock);

        nextPublishTime = Time.time + (1f / publishRateHz);
    }
}
