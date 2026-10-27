using UnityEngine;

public class servo_RB : MonoBehaviour
{
    [Header("Rotation")]
    [Tooltip("Vitesse de la consigne (°/s)")]
    public float vitesse = 5f;

    [Tooltip("Course maximale par rapport à la position de départ (°)")]
    public float angleMax = 50f;

    [Header("Touches")]
    public KeyCode toucheDroite = KeyCode.RightArrow;    // ou KeyCode.W
    public KeyCode toucheGauche = KeyCode.LeftArrow; // ou KeyCode.S

    private ConfigurableJoint joint;
    private Vector3 rotationDepart; // rotation locale initiale
    private float thConsigne;        // déplacement cible sur Z (relatif au départ)

    void Start()
    {
        joint = GetComponent<ConfigurableJoint>();
        rotationDepart = transform.localRotation.eulerAngles;
        thConsigne = 0f;

        ConfigurerDrive();

    }

    /// <summary>
    /// Active un JointDrive sur l'axe Y pour que le joint suive targetRotation.
    /// </summary>
    private void ConfigurerDrive()
    {
        var drive = new JointDrive
        {
            positionSpring = 2000f,   // raideur : plus grand = suivi plus net
            positionDamper = 60f,    // amortissement : évite l'oscillation
            maximumForce = 5000f     // force max que le drive peut appliquer
        };
        joint.angularYZDrive = drive;
        // targetPosition est exprimé dans l'espace du joint :
        // 0 = position au moment de Start
        joint.targetRotation = Quaternion.identity;
    }

    void Update()
    {
        float direction = 0f;
        if (Input.GetKey(toucheDroite)) direction -= 1f;
        if (Input.GetKey(toucheGauche)) direction += 1f;

        if (direction != 0f)
        {
            thConsigne = Mathf.Clamp(thConsigne + direction * vitesse * Time.deltaTime,
                                    -angleMax, angleMax);
        }

        // Le joint amène la masse vers la consigne (relatif à la position de référence)
        
        joint.targetRotation = Quaternion.Euler(0f,thConsigne,0f);
    }
}
