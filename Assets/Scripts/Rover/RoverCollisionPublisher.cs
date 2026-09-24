using System;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

// Attach ONE publisher to the rover root. Relay contacts from every physics body.
public class RoverCollisionPublisher : MonoBehaviour
{
    public string topic = "/test/collision_status";
    [Tooltip("Include obstacle/wall layers. Exclude terrain, ground and rover layers.")]
    public LayerMask obstacleLayers;
    public Collider[] ignoredColliders = new Collider[0];
    public float heartbeatSeconds = 0.1f;

    [Serializable]
    class Status
    {
        public string session;
        public int count;
        public int active;
        public bool configured;
        public string last_object = "";
        public float last_contact_time;
        public float last_relative_speed;
    }

    readonly HashSet<string> contacts = new HashSet<string>();
    readonly Status status = new Status();
    ROSConnection ros;
    float nextHeartbeat;

    void Start()
    {
        status.session = Guid.NewGuid().ToString();
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<StringMsg>(topic);
        Publish();
    }

    public void Contact(int sourceId, Collision collision, bool touching)
    {
        Collider obstacle = collision.collider;
        if (obstacle.transform.IsChildOf(transform)) return;
        if (Array.IndexOf(ignoredColliders, obstacle) >= 0) return;
        if ((obstacleLayers.value & (1 << obstacle.gameObject.layer)) == 0) return;
        string key = sourceId + ":" + obstacle.GetInstanceID();
        if (touching && contacts.Add(key))
        {
            status.count++;
            status.last_object = obstacle.name;
            status.last_contact_time = Time.time;
            status.last_relative_speed = collision.relativeVelocity.magnitude;
            Publish();
        }
        else if (!touching && contacts.Remove(key)) Publish();
    }

    void FixedUpdate()
    {
        if (Time.time >= nextHeartbeat)
        {
            Publish();
            nextHeartbeat = Time.time + Mathf.Max(0.02f, heartbeatSeconds);
        }
    }

    void Publish()
    {
        if (ros == null) return;
        status.active = contacts.Count;
        bool hasRelay = false;
        foreach (var relay in GetComponentsInChildren<RoverCollisionRelay>())
            if (relay.isActiveAndEnabled && relay.publisher == this) hasRelay = true;
        status.configured = obstacleLayers.value != 0 && hasRelay;
        ros.Publish(topic, new StringMsg(JsonUtility.ToJson(status)));
    }
}
