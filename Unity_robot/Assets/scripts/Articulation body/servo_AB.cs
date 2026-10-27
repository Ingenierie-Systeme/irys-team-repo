using UnityEngine;

/// <summary>
/// Contrôle au clavier la rotation d'une articulation revolute.
/// </summary>
[RequireComponent(typeof(ArticulationBody))]
public class servo_AB : MonoBehaviour
{
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

        thConsigne = 0f;
        ConfigurerDrive();
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

    void FixedUpdate()
    {
        thConsigne = Mathf.Clamp(thConsigne + direction * vitesse * Time.fixedDeltaTime,
                                 -angleMax, angleMax);

        ArticulationDrive drive = articulationBody.xDrive;
        drive.target = thConsigne;
        articulationBody.xDrive = drive;
    }
}
