using UnityEngine;

// Attach to every Rigidbody/ArticulationBody that can contact obstacles.
// Use physical colliders, not triggers. Do not attach duplicate relays to a body.
public class RoverCollisionRelay : MonoBehaviour
{
    public RoverCollisionPublisher publisher;

    void Awake()
    {
        if (publisher == null) publisher = GetComponentInParent<RoverCollisionPublisher>();
        if (publisher == null) Debug.LogError("Collision relay needs a RoverCollisionPublisher", this);
    }

    void OnCollisionEnter(Collision collision) { if (publisher != null) publisher.Contact(GetInstanceID(), collision, true); }
    // Stay also registers an existing contact when the observer/publisher starts late.
    void OnCollisionStay(Collision collision) { if (publisher != null) publisher.Contact(GetInstanceID(), collision, true); }
    void OnCollisionExit(Collision collision) { if (publisher != null) publisher.Contact(GetInstanceID(), collision, false); }
}
