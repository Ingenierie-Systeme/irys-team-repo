using UnityEngine;

// Simulation d'un USV avec comportement de flottabilité par échantillonnage du volume.
// en utilisant un articulated body
public class USV_AB : MonoBehaviour
{
    public ArticulationBody articulationBody;

    public float waterDensity = 1000f; // densité de l'eau (kg/m^3)

    [Header("Flottabilité")]
    public Collider[] buoyancyColliders;
    public float waterLevel = 0f;
    [Range(4, 40)]
    public int samplesPerAxis = 8;

    [Header("Valeurs calculées")]
    [SerializeField] float submergedVolume;
    [SerializeField] Vector3 centerOfBuoyancy;

    void Start()
    {
        if (articulationBody == null)
            articulationBody = GetComponentInParent<ArticulationBody>();

        if (articulationBody == null)
        {
            Debug.LogError("USV_AB nécessite une ArticulationBody assignée ou présente sur elle-même/un parent.", this);
            enabled = false;
            return;
        }

        if (samplesPerAxis <= 0)
        {
            Debug.LogError("samplesPerAxis doit être supérieur à zéro.", this);
            enabled = false;
            return;
        }

        if (buoyancyColliders == null || buoyancyColliders.Length == 0)
            buoyancyColliders = GetComponentsInChildren<Collider>();
    }

    private void AddBuoyancyForce()
    {
        submergedVolume = 0f;
        Vector3 weightedCenter = Vector3.zero;

        foreach (Collider collider in buoyancyColliders)
        {
            if (collider == null || collider.isTrigger)
                continue;

            Bounds bounds = collider.bounds;
            Vector3 cellSize = bounds.size / samplesPerAxis;
            float cellVolume = cellSize.x * cellSize.y * cellSize.z;

            for (int x = 0; x < samplesPerAxis; x++)
            {
                for (int y = 0; y < samplesPerAxis; y++)
                {
                    for (int z = 0; z < samplesPerAxis; z++)
                    {
                        Vector3 point = bounds.min + new Vector3(
                            (x + 0.5f) * cellSize.x,
                            (y + 0.5f) * cellSize.y,
                            (z + 0.5f) * cellSize.z);

                        if (point.y > waterLevel || !IsInsideCollider(collider, point))
                            continue;

                        submergedVolume += cellVolume;
                        weightedCenter += point * cellVolume;
                    }
                }
            }
        }

        if (submergedVolume <= 0f)
        {
            centerOfBuoyancy = transform.position;
            return;
        }

        centerOfBuoyancy = weightedCenter / submergedVolume;

        float buoyancyMagnitude = waterDensity * Physics.gravity.magnitude * submergedVolume;
        articulationBody.AddForceAtPosition(Vector3.up * buoyancyMagnitude,
            centerOfBuoyancy, ForceMode.Force);
    }

    private bool IsInsideCollider(Collider collider, Vector3 point)
    {
        Vector3 closestPoint = collider.ClosestPoint(point);
        return (closestPoint - point).sqrMagnitude < 0.00000001f;
    }

    void FixedUpdate()
    {
        AddBuoyancyForce();
    }
}
