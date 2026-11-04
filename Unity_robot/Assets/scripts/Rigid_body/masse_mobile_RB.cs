using UnityEngine;

/// <summary>
/// Contrôle la position d'une masse mobile sur l'axe Z via un ConfigurableJoint,
/// piloté au clavier.
///
/// Configuration attendue du ConfigurableJoint :
///   - ZMotion  : Limited (limites ±0.1 sur Z)
///   - XMotion / YMotion / toutes les rotations angulaires : Locked

/// </summary>
[RequireComponent(typeof(ConfigurableJoint))]
public class MasseMobileController_RB : MonoBehaviour
{
    [Header("Déplacement")]
    [Tooltip("Vitesse de la consigne (m/s)")]
    public float vitesse = 0.1f;

    [Tooltip("Course maximale par rapport à la position de départ (m)")]
    public float courseMax = 0.1f;

    [Header("Touches")]
    public KeyCode toucheAvant = KeyCode.UpArrow;    // ou KeyCode.W
    public KeyCode toucheArriere = KeyCode.DownArrow; // ou KeyCode.S

    private ConfigurableJoint joint;
    private Vector3 positionDepart; // position locale initiale
    private float zConsigne;        // déplacement cible sur Z (relatif au départ)

    void Start()
    {
        joint = GetComponent<ConfigurableJoint>();
        positionDepart = transform.localPosition;
        zConsigne = 0f;

        ConfigurerDrive();

    }

    /// <summary>
    /// Active un JointDrive sur l'axe Z pour que le joint suive targetPosition.
    /// </summary>
    private void ConfigurerDrive()
    {
        var drive = new JointDrive
        {
            positionSpring = 500f,   // raideur : plus grand = suivi plus net
            positionDamper = 30f,    // amortissement : évite l'oscillation
            maximumForce = 1000f     // force max que le drive peut appliquer
        };
        joint.zDrive = drive;
        // targetPosition est exprimé dans l'espace du joint :
        // 0 = position au moment de Start
        joint.targetPosition = Vector3.zero;
    }

    void Update()
    {
        float direction = 0f;
        if (Input.GetKey(toucheAvant)) direction -= 1f;
        if (Input.GetKey(toucheArriere)) direction += 1f;

        if (direction != 0f)
        {
            zConsigne = Mathf.Clamp(zConsigne + direction * vitesse * Time.deltaTime,
                                    -courseMax, courseMax);
        }

        // Le joint amène la masse vers la consigne (relatif à la position de référence)
        joint.targetPosition = new Vector3(0f, 0f, zConsigne);
    }
}