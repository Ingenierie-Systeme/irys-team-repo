using UnityEngine;

// Physique des articulation body
[RequireComponent(typeof(ArticulationBody))]
public class modif_damping : MonoBehaviour
{
    [Header("Amortissement des translations")]
    [Tooltip("Sur un joint Fixed, coefficient de force de damping selon l'axe local X; sinon damping du drive X.")]
    [Min(0f)]
    [SerializeField] private float amortissementLineaireX = 0f;

    [Tooltip("Sur un joint Fixed, coefficient de force de damping selon l'axe local Y; sinon damping du drive Y.")]
    [Min(0f)]
    [SerializeField] private float amortissementLineaireY = 0f;

    [Tooltip("Sur un joint Fixed, coefficient de force de damping selon l'axe local Z; sinon damping du drive Z.")]
    [Min(0f)]
    [SerializeField] private float amortissementLineaireZ = 0f;

    [Header("Amortissement angulaire")]
    [Tooltip("Sur un joint Fixed, coefficient de couple de damping selon l'axe local X; sinon damping du twistDrive.")]
    [Min(0f)]
    [SerializeField] private float amortissementAngulaireX = 0f;

    [Tooltip("Sur un joint Fixed, coefficient de couple de damping selon l'axe local Y; sinon damping du swingYDrive.")]
    [Min(0f)]
    [SerializeField] private float amortissementAngulaireY = 0f;

    [Tooltip("Sur un joint Fixed, coefficient de couple de damping selon l'axe local Z; sinon damping du swingZDrive.")]
    [Min(0f)]
    [SerializeField] private float amortissementAngulaireZ = 0f;

    private ArticulationBody articulationBody;

    private void Awake()
    {
        articulationBody = GetComponent<ArticulationBody>();
    }

    private void Start()
    {
        if (!AmortissementValide(amortissementLineaireX) ||
            !AmortissementValide(amortissementLineaireY) ||
            !AmortissementValide(amortissementLineaireZ) ||
            !AmortissementValide(amortissementAngulaireX) ||
            !AmortissementValide(amortissementAngulaireY) ||
            !AmortissementValide(amortissementAngulaireZ))
        {
            Debug.LogError("Les amortissements doivent être finis et non négatifs.", this);
            enabled = false;
            return;
        }
    }

    private void FixedUpdate()
    {
        if (articulationBody.jointType != ArticulationJointType.FixedJoint)
            return;

        Vector3 vitesseLocale = transform.InverseTransformDirection(articulationBody.linearVelocity);
        Vector3 forceLocale = new Vector3(
            -amortissementLineaireX * vitesseLocale.x,
            -amortissementLineaireY * vitesseLocale.y,
            -amortissementLineaireZ * vitesseLocale.z);
        articulationBody.AddForce(transform.TransformDirection(forceLocale), ForceMode.Force);

        Vector3 vitesseAngulaireLocale =
            transform.InverseTransformDirection(articulationBody.angularVelocity);
        Vector3 coupleLocal = new Vector3(
            -amortissementAngulaireX * vitesseAngulaireLocale.x,
            -amortissementAngulaireY * vitesseAngulaireLocale.y,
            -amortissementAngulaireZ * vitesseAngulaireLocale.z);
        articulationBody.AddTorque(transform.TransformDirection(coupleLocal), ForceMode.Force);
    }

    private static bool AmortissementValide(float valeur)
    {
        return valeur >= 0f && !float.IsNaN(valeur) && !float.IsInfinity(valeur);
    }
}
