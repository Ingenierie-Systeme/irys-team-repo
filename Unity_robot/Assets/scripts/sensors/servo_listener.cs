using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

/// <summary>
/// Écoute un topic ROS de consigne d'angle et l'applique au servo Unity.
/// Topic attendu : std_msgs/Float32MultiArray, avec l'angle en degrés
/// à l'index configuré.
/// </summary>
public class ServoROSListener : MonoBehaviour
{
    [Header("ROS")]
    [Tooltip("Nom du topic ROS contenant les consignes des servos")]
    [SerializeField] private string topicName = "/output/servos";

    [Tooltip(
        "Index de la consigne dans le tableau (0 : nageoire, 1 : masse mobile)."
    )]
    [SerializeField] private int commandIndex;

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
            Debug.LogError(
                "ServoROSListener nécessite un composant servo_AB sur "
                + "lui-même ou sur un parent.",
                this
            );
            enabled = false;
            return;
        }

        if (commandIndex < 0)
        {
            Debug.LogError("L'index de commande du servo ne peut pas être négatif.", this);
            enabled = false;
            return;
        }

        ros.Subscribe<Float32MultiArrayMsg>(topicName, OnServoCommands);
    }

    private void OnServoCommands(Float32MultiArrayMsg message)
    {
        if (message.data == null || message.data.Length <= commandIndex)
        {
            Debug.LogWarning(
                $"Le tableau {topicName} ne contient pas la commande d'index {commandIndex}.",
                this
            );
            return;
        }

        float angle = message.data[commandIndex];
        if (angleInRadians)
            servo.SetAngleCommandRadians(angle);
        else
            servo.SetAngleCommand(angle);
    }
}
