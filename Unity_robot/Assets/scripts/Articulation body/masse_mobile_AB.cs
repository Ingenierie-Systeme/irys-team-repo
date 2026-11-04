using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

/// <summary>
/// Contrôle la position d'une masse mobile depuis une consigne ROS en degrés.
/// </summary>
[RequireComponent(typeof(ArticulationBody))]
public class masse_mobile_AB : MonoBehaviour
{
    [Header("ROS")]
    [SerializeField] private string topicName = "/servo/masse_cmd";

    [Tooltip("Valeur absolue maximale de la commande ROS, en degrés.")]
    [SerializeField] private float commandeMaxDegrees = 50f;

    [Tooltip("Délai avant recentrage si la commande ROS cesse d'arriver (s).")]
    [SerializeField] private float delaiCommandeSecondes = 1f;

    [Header("Déplacement")]
    [Tooltip("Vitesse maximale de déplacement de la masse (m/s).")]
    public float vitesse = 0.1f;

    [Tooltip("Course maximale par rapport à la position de départ (m)")]
    public float courseMax = 0.1f;

    private ArticulationBody articulationBody;
    private float zConsigne;
    private float zVoulue;
    private float derniereCommandeTime;
    private bool commandeRecue;

    void Start()
    {
        articulationBody = GetComponent<ArticulationBody>();
        if (articulationBody.jointType != ArticulationJointType.PrismaticJoint)
        {
            Debug.LogError(
                "La masse mobile doit utiliser une ArticulationBody "
                + "de type Prismatic.",
                this
            );
            enabled = false;
            return;
        }

        if (float.IsNaN(courseMax) || float.IsInfinity(courseMax)
            || float.IsNaN(vitesse) || float.IsInfinity(vitesse)
            || float.IsNaN(commandeMaxDegrees)
            || float.IsInfinity(commandeMaxDegrees)
            || float.IsNaN(delaiCommandeSecondes)
            || float.IsInfinity(delaiCommandeSecondes)
            || courseMax <= 0f || vitesse <= 0f
            || commandeMaxDegrees <= 0f
            || delaiCommandeSecondes <= 0f)
        {
            Debug.LogError(
                "courseMax, vitesse, commandeMaxDegrees et "
                + "delaiCommandeSecondes doivent être supérieurs à zéro.",
                this
            );
            enabled = false;
            return;
        }

        zConsigne = 0f;
        zVoulue = 0f;
        ConfigurerDrive();

        ROSConnection ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<Float64Msg>(topicName, OnPositionMessage);
        Debug.Log($"La masse mobile écoute le topic ROS {topicName}.", this);
    }

    private void ConfigurerDrive()
    {
        articulationBody.linearLockX = ArticulationDofLock.LockedMotion;
        articulationBody.linearLockY = ArticulationDofLock.LockedMotion;
        articulationBody.linearLockZ = ArticulationDofLock.LimitedMotion;

        ArticulationDrive drive = articulationBody.zDrive;
        drive.stiffness = 500f;
        drive.damping = 30f;
        drive.forceLimit = 1000f;
        drive.lowerLimit = -courseMax;
        drive.upperLimit = courseMax;
        drive.target = zConsigne;
        articulationBody.zDrive = drive;
    }

    private void OnPositionMessage(Float64Msg message)
    {
        float commande = (float)message.data;
        if (float.IsNaN(commande) || float.IsInfinity(commande))
        {
            Debug.LogWarning("La commande de masse doit être finie.", this);
            return;
        }

        float commandeLimitee = Mathf.Clamp(
            commande,
            -commandeMaxDegrees,
            commandeMaxDegrees
        );
        zVoulue = commandeLimitee / commandeMaxDegrees * courseMax;
        derniereCommandeTime = Time.realtimeSinceStartup;
        commandeRecue = true;
    }

    void FixedUpdate()
    {
        if (!commandeRecue
            || Time.realtimeSinceStartup - derniereCommandeTime
                > delaiCommandeSecondes)
        {
            zVoulue = 0f;
        }

        zConsigne = Mathf.MoveTowards(
            zConsigne,
            zVoulue,
            vitesse * Time.fixedDeltaTime
        );

        ArticulationDrive drive = articulationBody.zDrive;
        drive.target = zConsigne;
        articulationBody.zDrive = drive;
    }
}
