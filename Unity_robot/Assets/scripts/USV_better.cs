using UnityEngine;
using UnityEngine.InputSystem;

//simulation d'un USV avec comportement de flotabilité plus réaliste
//  et propulsion par deux propulseurs (gauche/droite) avec contrôle de vitesse et de rotation
// Les propulseurs doivent être crées en tant qu'enfants de l'USV et placés aux positions désirées sur l'USV
// L'USV nécessite d'avoir un RigidBody pour l'application des forces et un ou plusieurs colliders pour le calcul de la flottaison

public class USV_better : MonoBehaviour
{
    public Rigidbody rb;
    public Transform tl;
    public Transform tr;
    public bool useKeyboard = true; // activer contrôles clavier

    public float waterDensity = 1000f; // densité de l'eau (kg/m^3)

    [Header("Flottabilité")]
    public Collider[] buoyancyColliders;
    public float waterLevel = 0f;
    [Range(4, 40)]
    public int samplesPerAxis = 16;


    [Header("Valeurs calculées")]
    [SerializeField] float submergedVolume;
    [SerializeField] Vector3 centerOfBuoyancy;

    [Header("Paramètres de propulsion")]
    public float v=300f; // vitesse max (m/s)
    public float f=80f; // force de rotation max (N)

    private float u1; // vitesse commandée (m/s)
    private float u2; // commande de rotation (rad/s)
    private Vector3 LThrust;
    private Vector3 RThrust;
    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        if (rb == null)
            rb = GetComponent<Rigidbody>();

        if (buoyancyColliders == null || buoyancyColliders.Length == 0)
            buoyancyColliders = GetComponentsInChildren<Collider>();


        u1 = 0f; // vitesse commandée (m/s)
        u2 = 0f; // commande de rotation (rad/s)
        LThrust = Vector3.zero;
        RThrust = Vector3.zero;
    }
    private void add_buoyancy_force()
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
        rb.AddForceAtPosition(Vector3.up * buoyancyMagnitude,
            centerOfBuoyancy, ForceMode.Force);
    }

    bool IsInsideCollider(Collider collider, Vector3 point)
    {
        Vector3 closestPoint = collider.ClosestPoint(point);
        return (closestPoint - point).sqrMagnitude < 0.00000001f;
    }

    private void add_thruster_force()
    {
        LThrust = (u1 - u2) * tl.forward;
        RThrust = (u1 + u2) * tr.forward;
        rb.AddForceAtPosition(LThrust, tl.position, ForceMode.Force);
        rb.AddForceAtPosition(RThrust, tr.position, ForceMode.Force);
    }
    // Update is called once per frame
    void FixedUpdate()
    {
        add_buoyancy_force();
        if (useKeyboard)
        {
            // up = avancer, down = arrêter / reculer (si forwardOnly=false)
            if (Input.GetKey(KeyCode.UpArrow)) u1 = v;
            else if (Input.GetKey(KeyCode.DownArrow)) u1 = -v;
            else u1 = 0f;

            // left/right = tourner (entrée binaire simple)
            if (Input.GetKey(KeyCode.LeftArrow)) u2 = f; // tourner à gauche
            else if (Input.GetKey(KeyCode.RightArrow)) u2 = -f; // tourner à droite
            else u2 = 0f;
        }
        add_thruster_force();
    }
}
