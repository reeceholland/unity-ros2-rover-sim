using UnityEngine;

public class UnityTopicRemapper : MonoBehaviour
{
    public enum TopicKind
    {
        Sensor,
        ActuatorFeedback,
        MotorCommand,
        Other
    }

    [Header("Fault Injection")]
    public bool Remap = false;
    public string rawPostfix = "_raw";

    [Header("Topic Groups")]
    public bool remapSensorTopics = true;
    public bool remapActuatorFeedbackTopics = true;
    public bool remapMotorCommandTopics = false;

    public string Resolve(string topic, TopicKind kind)
    {
        if (!Remap || string.IsNullOrWhiteSpace(topic))
        {
            return topic;
        }

        if (!ShouldRemap(kind) || string.IsNullOrWhiteSpace(rawPostfix) || topic.EndsWith(rawPostfix))
        {
            return topic;
        }

        return $"{topic}{rawPostfix}";
    }

    bool ShouldRemap(TopicKind kind)
    {
        switch (kind)
        {
            case TopicKind.Sensor:
                return remapSensorTopics;
            case TopicKind.ActuatorFeedback:
                return remapActuatorFeedbackTopics;
            case TopicKind.MotorCommand:
                return remapMotorCommandTopics;
            default:
                return false;
        }
    }

    public static string Resolve(Component source, string topic, TopicKind kind)
    {
        UnityTopicRemapper remapper = null;

        if (source != null)
        {
            remapper = source.GetComponentInParent<UnityTopicRemapper>();
        }

        if (remapper == null)
        {
            remapper = FindObjectOfType<UnityTopicRemapper>();
        }

        return remapper == null ? topic : remapper.Resolve(topic, kind);
    }
}
