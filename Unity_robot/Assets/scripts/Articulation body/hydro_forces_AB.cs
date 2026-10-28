using UnityEngine;

// Calcul des forces hydrodynamiques appliquées sur un segment de nageoire.

[RequireComponent(typeof(MeshFilter))]
public class Nageoire_AB : MonoBehaviour
{
    [SerializeField] private HydroForcesSettings settings;

    [Header("Valeurs calculées")]
    [SerializeField] private float surfaceParPanneau;

    private ArticulationBody articulationBody;
    private Bounds boundsMaillage;
    private Vector3 axeEnvergureMonde;
    private Vector3 axeCordeMonde;
    private Vector3 normaleMonde;

    void Awake()
    {
        if (settings == null)
        {
            Debug.LogError("Assignez un HydroForcesSettings partagé à Nageoire_AB.", this);
            enabled = false;
            return;
        }

        articulationBody = GetComponentInParent<ArticulationBody>();
        if (articulationBody == null)
        {
            Debug.LogError("Nageoire_AB nécessite une ArticulationBody sur elle-même ou un parent.", this);
            enabled = false;
            return;
        }

        Mesh mesh = GetComponent<MeshFilter>().sharedMesh;
        if (mesh == null)
        {
            Debug.LogError("Nageoire_AB nécessite un maillage sur le MeshFilter.", this);
            enabled = false;
            return;
        }

        if (settings.nPanels <= 0 || settings.echantillonsImmersion <= 0 ||
            settings.densiteEau < 0f || settings.coefTraineeNormale < 0f ||
            settings.coefTraineeTangentielle < 0f || settings.facteurPortance < 0f ||
            settings.angleDecrochage <= 0f || settings.articulationStiffness < 0f ||
            settings.articulationDamping < 0f)
        {
            Debug.LogError("Les nombres d'échantillons doivent être positifs et les coefficients, stiffness et damping non négatifs.", this);
            enabled = false;
            return;
        }

        boundsMaillage = mesh.bounds;
        if (boundsMaillage.size.y <= 0f || boundsMaillage.size.z <= 0f)
        {
            Debug.LogError("Le maillage de la nageoire doit avoir une surface non nulle dans le plan local Y-Z.", this);
            enabled = false;
            return;
        }

        if (!ActualiserGeometrieMonde())
        {
            Debug.LogError("L'échelle ou l'orientation de la nageoire produit une surface nulle.", this);
            enabled = false;
        }
        articulationBody.xDrive = ConfigurerDrive(articulationBody.xDrive);
        articulationBody.yDrive = ConfigurerDrive(articulationBody.yDrive);
        articulationBody.zDrive = ConfigurerDrive(articulationBody.zDrive);
    }

    private ArticulationDrive ConfigurerDrive(ArticulationDrive drive)
    {
        drive.stiffness = settings.articulationStiffness;
        drive.damping = settings.articulationDamping;
        return drive;
    }

    void FixedUpdate()
    {
        if (!ActualiserGeometrieMonde())
            return;

        float largeurPanneau = boundsMaillage.size.z / settings.nPanels;

        for (int i = 0; i < settings.nPanels; i++)
        {
            float zLocal = boundsMaillage.min.z + (i + 0.5f) * largeurPanneau;
            Vector3 positionMoyenneImmergee = Vector3.zero;
            int pointsImmerges = 0;

            for (int j = 0; j < settings.echantillonsImmersion; j++)
            {
                float fractionEnvergure = (j + 0.5f) / settings.echantillonsImmersion;
                float yLocal = boundsMaillage.min.y + fractionEnvergure * boundsMaillage.size.y;
                Vector3 point = transform.TransformPoint(
                    new Vector3(boundsMaillage.center.x, yLocal, zLocal));

                if (point.y <= settings.waterLevel)
                {
                    positionMoyenneImmergee += point;
                    pointsImmerges++;
                }
            }

            if (pointsImmerges == 0)
                continue;

            float fractionImmergee = (float)pointsImmerges / settings.echantillonsImmersion;
            float surfaceImmergee = surfaceParPanneau * fractionImmergee;
            Vector3 position = positionMoyenneImmergee / pointsImmerges;
            Vector3 vitesseRelative = articulationBody.GetPointVelocity(position) - settings.waterVelocity;

            float vitesseNormale = Vector3.Dot(vitesseRelative, normaleMonde);
            Vector3 vitesseTangentielle = vitesseRelative - vitesseNormale * normaleMonde;
            float pression = 0.5f * settings.densiteEau * surfaceImmergee;

            Vector3 force = -pression * settings.coefTraineeNormale *
                            vitesseNormale * Mathf.Abs(vitesseNormale) * normaleMonde;
            force -= pression * settings.coefTraineeTangentielle *
                     vitesseTangentielle.magnitude * vitesseTangentielle;

            if (settings.utiliserPortance && vitesseRelative.sqrMagnitude > 1e-8f)
                force += CalculerPortance(pression, vitesseRelative);

            articulationBody.AddForceAtPosition(force, position, ForceMode.Force);
        }
    }

    private Vector3 CalculerPortance(float pression, Vector3 vitesseRelative)
    {
        Vector3 ecoulement = -vitesseRelative.normalized;
        float composanteCorde = Vector3.Dot(ecoulement, axeCordeMonde.normalized);
        float composanteNormale = Vector3.Dot(ecoulement, normaleMonde);
        float angleAttaque = Mathf.Atan2(composanteNormale, composanteCorde);
        float angleLimite = settings.angleDecrochage * Mathf.Deg2Rad;
        float coefficient = settings.facteurPortance * 2f * Mathf.PI *
                            Mathf.Clamp(angleAttaque, -angleLimite, angleLimite);

        Vector3 direction = Vector3.Cross(axeEnvergureMonde.normalized, ecoulement);
        if (direction.sqrMagnitude <= 1e-8f)
            return Vector3.zero;

        return pression * coefficient * vitesseRelative.sqrMagnitude * direction.normalized;
    }

    private bool ActualiserGeometrieMonde()
    {
        axeEnvergureMonde = transform.TransformVector(Vector3.up);
        axeCordeMonde = transform.TransformVector(Vector3.forward);
        Vector3 produitVectoriel = Vector3.Cross(axeEnvergureMonde, axeCordeMonde);

        if (produitVectoriel.sqrMagnitude <= Mathf.Epsilon)
            return false;

        normaleMonde = produitVectoriel.normalized;
        surfaceParPanneau = boundsMaillage.size.y * boundsMaillage.size.z *
                            produitVectoriel.magnitude / settings.nPanels;
        return surfaceParPanneau > 0f;
    }
}
