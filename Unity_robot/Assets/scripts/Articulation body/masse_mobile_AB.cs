using UnityEngine;

/// <summary>
/// Contrôle au clavier la position d'une masse mobile sur l'axe Z d'une articulation prismatique.
/// </summary>
[RequireComponent(typeof(ArticulationBody))]
public class masse_mobile_AB : MonoBehaviour
{
    [Header("Déplacement")]
    [Tooltip("Vitesse de la consigne (m/s)")]
    public float vitesse = 0.1f;

    [Tooltip("Course maximale par rapport à la position de départ (m)")]
    public float courseMax = 0.1f;

    [Header("Touches")]
    public KeyCode toucheAvant = KeyCode.UpArrow;
    public KeyCode toucheArriere = KeyCode.DownArrow;

    private ArticulationBody articulationBody;
    private float zConsigne;
    private float direction;

    void Start()
    {
        articulationBody = GetComponent<ArticulationBody>();
        if (articulationBody.jointType != ArticulationJointType.PrismaticJoint)
        {
            Debug.LogError("La masse mobile doit utiliser une ArticulationBody de type Prismatic.", this);
            enabled = false;
            return;
        }

        if (courseMax <= 0f)
        {
            Debug.LogError("courseMax doit être supérieur à zéro.", this);
            enabled = false;
            return;
        }

        zConsigne = 0f;
        ConfigurerDrive();
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

    void Update()
    {
        direction = 0f;
        if (Input.GetKey(toucheAvant)) direction += 1f;
        if (Input.GetKey(toucheArriere)) direction -= 1f;
    }

    void FixedUpdate()
    {
        zConsigne = Mathf.Clamp(zConsigne + direction * vitesse * Time.fixedDeltaTime,
                                -courseMax, courseMax);

        ArticulationDrive drive = articulationBody.zDrive;
        drive.target = zConsigne;
        articulationBody.zDrive = drive;
    }
}
