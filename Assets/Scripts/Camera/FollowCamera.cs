using UnityEngine;

public class FollowCamera : MonoBehaviour
{
    public Transform target;
    public Vector3 offset = new Vector3(0f, 2.5f, -5f);
    public float positionSmoothTime = 0.15f;
    public float rotationSmoothSpeed = 8f;
    public bool lookAtTarget = true;
    public Vector3 lookAtOffset = new Vector3(0f, 0.4f, 0f);

    Vector3 velocity;

    void LateUpdate()
    {
        if (target == null)
        {
            return;
        }

        Vector3 desiredPosition = target.TransformPoint(offset);
        transform.position = Vector3.SmoothDamp(
            transform.position,
            desiredPosition,
            ref velocity,
            positionSmoothTime);

        if (!lookAtTarget)
        {
            return;
        }

        Vector3 lookTarget = target.position + lookAtOffset;
        Vector3 lookDirection = lookTarget - transform.position;
        if (lookDirection.sqrMagnitude < 0.0001f)
        {
            return;
        }

        Quaternion desiredRotation = Quaternion.LookRotation(lookDirection.normalized, Vector3.up);
        transform.rotation = Quaternion.Slerp(
            transform.rotation,
            desiredRotation,
            rotationSmoothSpeed * Time.deltaTime);
    }
}
