using System;
using System.Linq;
using UnityEngine;
using UnityEngine.SceneManagement;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Nav;
using RosMessageTypes.Std;
using RosMessageTypes.Geometry;

// Runtime installation keeps existing/customized CI scenes intact. Rebuild the player.
public class CiObserverTelemetry : MonoBehaviour
{
    ROSConnection ros;
    Rigidbody body;
    const string Topic = "/ci/ground_truth/odom";
    float nextPublish;

    [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
    static void Install()
    {
        if (SceneManager.GetActiveScene().name != "CiLidarWorld") return;
        var drives = UnityEngine.Object.FindObjectsByType<A4WD3Drive>(FindObjectsSortMode.None);
        if (drives.Length != 1) throw new InvalidOperationException("CI observer requires exactly one rover");
        var rigidbody = drives[0].GetComponentInParent<Rigidbody>();
        if (rigidbody == null) rigidbody = drives[0].GetComponentInChildren<Rigidbody>();
        if (rigidbody == null) throw new InvalidOperationException("CI rover needs a Rigidbody for ground truth");
        var root = drives[0].gameObject;
        var telemetry = root.GetComponent<CiObserverTelemetry>() ?? root.AddComponent<CiObserverTelemetry>();
        telemetry.body = rigidbody;
        var publisher = root.GetComponent<RoverCollisionPublisher>() ?? root.AddComponent<RoverCollisionPublisher>();
        publisher.obstacleLayers = ~0;
        var floor = GameObject.Find("Floor");
        if (floor == null) throw new InvalidOperationException("CI Floor missing; collision exclusions must be explicit");
        publisher.ignoredColliders = floor.GetComponentsInChildren<Collider>();
        var bodies = root.GetComponentsInChildren<Rigidbody>();
        foreach (var target in bodies.Select(b => b.gameObject).Concat(
                     root.GetComponentsInChildren<ArticulationBody>().Select(b => b.gameObject)).Distinct())
        {
            var relay = target.GetComponent<RoverCollisionRelay>() ?? target.AddComponent<RoverCollisionRelay>();
            relay.publisher = publisher;
        }
        Debug.Log("CI_OBSERVER_TELEMETRY collision=/test/collision_status ground_truth=" + Topic);
    }

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<OdometryMsg>(Topic);
    }

    void FixedUpdate()
    {
        if (ros == null || body == null || Time.time < nextPublish) return;
        nextPublish = Time.time + 0.05f;
        var position = body.position;
        var rotation = body.rotation;
        // Odometry pose is world-frame; twist is child/body-frame per ROS convention.
        var velocity = body.transform.InverseTransformDirection(body.linearVelocity);
        var angular = body.transform.InverseTransformDirection(body.angularVelocity);
        ros.Publish(Topic, new OdometryMsg
        {
            header = new HeaderMsg { stamp = RosTimeUtils.Now(), frame_id = "unity_world" },
            child_frame_id = "base_link",
            pose = new PoseWithCovarianceMsg
            {
                pose = new PoseMsg
                {
                    position = new PointMsg(position.z, -position.x, position.y),
                    orientation = new QuaternionMsg(rotation.z, -rotation.x, rotation.y, -rotation.w)
                },
                covariance = new double[36]
            },
            twist = new TwistWithCovarianceMsg
            {
                twist = new TwistMsg
                {
                    linear = new Vector3Msg(velocity.z, -velocity.x, velocity.y),
                    angular = new Vector3Msg(-angular.z, angular.x, -angular.y)
                },
                covariance = new double[36]
            }
        });
    }
}
