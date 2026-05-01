using UnityEngine;

public class A4WD3Drive : MonoBehaviour
{
    [Header("Wheel Colliders")]
    public WheelCollider frontLeft;
    public WheelCollider frontRight;
    public WheelCollider rearLeft;
    public WheelCollider rearRight;

    [Header("Visual Wheels")]
    public Transform frontLeftMesh;
    public Transform frontRightMesh;
    public Transform rearLeftMesh;
    public Transform rearRightMesh;

    [Header("Drive")]
    public float motorTorque = 2f;
    public float turnTorque = 1.5f;
    public float brakeTorque = 5f;

    Quaternion frontLeftOffset;
    Quaternion frontRightOffset;
    Quaternion rearLeftOffset;
    Quaternion rearRightOffset;

    void Start()
    {
        frontLeftOffset = GetWheelOffset(frontLeft, frontLeftMesh);
        frontRightOffset = GetWheelOffset(frontRight, frontRightMesh);
        rearLeftOffset = GetWheelOffset(rearLeft, rearLeftMesh);
        rearRightOffset = GetWheelOffset(rearRight, rearRightMesh);
    }

    void FixedUpdate()
    {
        float forward = 0f;
        float turn = 0f;

        if (Input.GetKey(KeyCode.W)) forward += 1f;
        if (Input.GetKey(KeyCode.S)) forward -= 1f;
        if (Input.GetKey(KeyCode.D)) turn += 1f;
        if (Input.GetKey(KeyCode.A)) turn -= 1f;

        float leftTorque = (forward * motorTorque) - (turn * turnTorque);
        float rightTorque = (forward * motorTorque) + (turn * turnTorque);

        ApplyWheel(frontLeft, leftTorque);
        ApplyWheel(rearLeft, leftTorque);
        ApplyWheel(frontRight, rightTorque);
        ApplyWheel(rearRight, rightTorque);

        bool braking = Mathf.Approximately(forward, 0f) && Mathf.Approximately(turn, 0f);
        SetBrake(braking ? brakeTorque : 0f);

        UpdateWheelVisual(frontLeft, frontLeftMesh, frontLeftOffset);
        UpdateWheelVisual(frontRight, frontRightMesh, frontRightOffset);
        UpdateWheelVisual(rearLeft, rearLeftMesh, rearLeftOffset);
        UpdateWheelVisual(rearRight, rearRightMesh, rearRightOffset);
    }

    Quaternion GetWheelOffset(WheelCollider collider, Transform mesh)
    {
        collider.GetWorldPose(out _, out Quaternion colliderRotation);
        return Quaternion.Inverse(colliderRotation) * mesh.rotation;
    }

    void ApplyWheel(WheelCollider wheel, float torque)
    {
        wheel.motorTorque = torque;
    }

    void SetBrake(float torque)
    {
        frontLeft.brakeTorque = torque;
        frontRight.brakeTorque = torque;
        rearLeft.brakeTorque = torque;
        rearRight.brakeTorque = torque;
    }

    void UpdateWheelVisual(WheelCollider collider, Transform mesh, Quaternion offset)
    {
        collider.GetWorldPose(out Vector3 position, out Quaternion rotation);
        mesh.position = position;
        mesh.rotation = rotation * offset;
    }
}
