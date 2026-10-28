using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

/// <summary>
/// Écoute un topic ROS de consigne d'angle et l'applique au servo Unity.
/// Topic attendu : std_msgs/Float64, ex. /servo_angle_cmd
/// </summary>
public class ServoROSListener : MonoBehaviour
{
    [Header("ROS")]
    [Tooltip("Nom du topic ROS de consigne (ex : /servo_angle_cmd)")]
    [SerializeField] private string topicName = "/servo_angle_cmd";

    [Tooltip("Mettre à true si la commande ROS est en radians, sinon en degrés.")]
    [SerializeField] private bool angleInRadians = false;

    [Header("Servo")]
    [SerializeField] private servo_AB servo;

    private ROSConnection ros;

    private void Awake()
    {
        ros = ROSConnection.GetOrCreateInstance();

        if (servo == null)
            servo = GetComponentInParent<servo_AB>();

        if (servo == null)
        {
            Debug.LogError("ServoROSListener nécessite un composant servo_AB sur lui-même ou sur un parent.", this);
            enabled = false;
            return;
        }

        ros.Subscribe<Float64Msg>(topicName, OnAngleMessage);
    }

    private void OnAngleMessage(Float64Msg message)
    {
        float angle = (float)message.data;
        if (angleInRadians)
            servo.SetAngleCommandRadians(angle);
        else
            servo.SetAngleCommand(angle);
    }
}
