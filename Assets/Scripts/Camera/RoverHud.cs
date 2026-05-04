using TMPro;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

public class RoverHud : MonoBehaviour
{
    public TMP_Text rosIp;
    public TMP_Text rosPort;
    public TMP_Text rosStatus;
    public TMP_Text linearVelocityText;
    public TMP_Text angularVelocityText;
    public Rigidbody roverRigidBody;
    
    ROSConnection ros;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
    }
    void Update()
    {
        if (roverRigidBody == null)
        {
            return;
        }
        if (linearVelocityText != null)
        {
            linearVelocityText.text = $"Linear Velocity: {roverRigidBody.linearVelocity.magnitude:F2} m/s";
        }
        if (angularVelocityText != null)
        {
            angularVelocityText.text = $"Angular Velocity: {roverRigidBody.angularVelocity.magnitude:F2} rad/s";
        }
        if (rosIp != null)
        {
            rosIp.text = $"ROS IP: {ros.RosIPAddress}";
        }
        if (rosPort != null)
        {
            rosPort.text = $"ROS Port: {ros.RosPort}";
        }
        if (rosStatus != null)
        {
            rosStatus.text = $"ROS Status: {(ros.HasConnectionError ? "Disconnected" : "Connected")}";
        }
    }
}
