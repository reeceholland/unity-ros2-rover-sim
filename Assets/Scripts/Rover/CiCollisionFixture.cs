using System;
using UnityEngine;

// Explicit opt-in validation only. Normal CI scenes are unchanged.
public class CiCollisionFixture : MonoBehaviour
{
    Rigidbody body;
    float contactAt;
    bool spawned;

    public static void Install(GameObject root, Rigidbody body, RoverCollisionPublisher publisher)
    {
        var args = Environment.GetCommandLineArgs();
        if (Array.IndexOf(args, "--ci-missing-collision") >= 0)
        {
            publisher.enabled = false;
            Debug.Log("CI_COLLISION_FIXTURE missing telemetry");
        }
        if (Array.IndexOf(args, "--ci-wall-contact") < 0) return;
        var fixture = root.AddComponent<CiCollisionFixture>();
        fixture.body = body;
        fixture.contactAt = Time.time + 15f;
        Debug.Log("CI_COLLISION_FIXTURE wall contact scheduled at " + fixture.contactAt);
    }

    void FixedUpdate()
    {
        if (spawned || Time.time < contactAt) return;
        spawned = true;
        // Overlap the chassis with a static wall to exercise Unity's real
        // contact callbacks, relays, ROS serialization and CI assertion path.
        var wall = GameObject.CreatePrimitive(PrimitiveType.Cube);
        wall.name = "CI_IntentionalContactWall";
        wall.transform.localScale = new Vector3(0.1f, 1f, 1f);
        wall.transform.position = body.position;
        Debug.Log("CI_COLLISION_FIXTURE intentional physical wall overlap");
    }
}
