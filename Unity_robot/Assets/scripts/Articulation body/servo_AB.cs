using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;


[RequireComponent(typeof(ArticulationBody))]
public class servo_AB : MonoBehaviour
{
    [Header("ROS")]
    [SerializeField] private string topicName = "/servo/nageoire_cmd";

    [Header("Rotation")]
    [Tooltip("Vitesse de la consigne (°/s)")]
    public float vitesse = 5f;

    [Tooltip("Course maximale par rapport à la position de départ (°)")]
    public float angleMax = 50f;

    [Header("Touches")]
    public KeyCode toucheDroite = KeyCode.RightArrow;
    public KeyCode toucheGauche = KeyCode.LeftArrow;

    private ArticulationBody articulationBody;
    private float thConsigne;
    private float direction;
    private float positionVoulue;
    private float decalageClavier;
    private ROSConnection ros;

    void Start()
    {
        articulationBody = GetComponent<ArticulationBody>();
        if (articulationBody.jointType != ArticulationJointType.RevoluteJoint)
        {
            Debug.LogError("Le servo doit utiliser une ArticulationBody de type Revolute.", this);
            enabled = false;
            return;
        }

        if (angleMax <= 0f)
        {
            Debug.LogError("angleMax doit être supérieur à zéro.", this);
            enabled = false;
            return;
        }

        if (vitesse <= 0f)
        {
            Debug.LogError("La vitesse doit être supérieure à zéro.", this);
            enabled = false;
            return;
        }

        thConsigne = 0f;
        positionVoulue = 0f;
        decalageClavier = 0f;
        ConfigurerDrive();

        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<Float64Msg>(topicName, OnAngleMessage);
    }

    private void ConfigurerDrive()
    {
        articulationBody.twistLock = ArticulationDofLock.LimitedMotion;
        articulationBody.swingYLock = ArticulationDofLock.LockedMotion;
        articulationBody.swingZLock = ArticulationDofLock.LockedMotion;

        ArticulationDrive drive = articulationBody.xDrive;
        drive.lowerLimit = -angleMax;
        drive.upperLimit = angleMax;
        drive.target = thConsigne;
        articulationBody.xDrive = drive;
    }

    void Update()
    {
        direction = 0f;
        if (Input.GetKey(toucheDroite)) direction -= 1f;
        if (Input.GetKey(toucheGauche)) direction += 1f;
    }

    public void SetAngleCommand(float angleDegrees)
    {
        if (float.IsNaN(angleDegrees) || float.IsInfinity(angleDegrees))
        {
            Debug.LogWarning("La consigne du servo doit être un angle fini.", this);
            return;
        }

        positionVoulue = Mathf.Clamp(angleDegrees, -angleMax, angleMax);
    }

    public void SetAngleCommandRadians(float angleRadians)
    {
        SetAngleCommand(angleRadians * Mathf.Rad2Deg);
    }

    private void OnAngleMessage(Float64Msg message)
    {
        SetAngleCommand((float)message.data);
    }

    void FixedUpdate()
    {
        float deltaTime = Time.fixedDeltaTime;
        decalageClavier = Mathf.Clamp(
            decalageClavier + direction * vitesse * deltaTime,
            -angleMax,
            angleMax
        );
        float cible = Mathf.Clamp(
            positionVoulue + decalageClavier,
            -angleMax,
            angleMax
        );
        thConsigne = Mathf.MoveTowards(
            thConsigne,
            cible,
            vitesse * deltaTime
        );

        ArticulationDrive drive = articulationBody.xDrive;
        drive.target = thConsigne;
        articulationBody.xDrive = drive;
    }
}
