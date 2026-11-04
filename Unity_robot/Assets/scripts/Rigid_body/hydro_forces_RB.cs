using UnityEngine;

public class Nageoire_RB : MonoBehaviour
{
    public int nPanels = 6; //décomposition de la surface en paneaux
    public float longueur = 0.25f;   // 25 cm
    public float hauteur = 0.15f;   // 15 cm
    public float densiteEau = 1000f;
    public float coefTrainee = 1.15f;

    private Rigidbody rb;
    private Vector3[] previousPanelPositions;
    private bool hasPreviousPanelPositions;

    void Awake()
    {
        rb = GetComponentInParent<Rigidbody>();
        if (rb == null)
        {
            Debug.LogError("Nageoire nécessite un Rigidbody sur elle-même ou un parent.", this);
            enabled = false;
            return;
        }

        if (nPanels <= 0)
        {
            Debug.LogError("Le nombre de panneaux de la nageoire doit être supérieur à zéro.", this);
            enabled = false;
            return;
        }

        previousPanelPositions = new Vector3[nPanels];
    }

    void FixedUpdate()
    {
        float sPanel = longueur * hauteur / nPanels;

        for (int i = 0; i < nPanels; i++)
        {
            float x = longueur * (i + 0.5f) / nPanels;
            Vector3 p = transform.TransformPoint(new Vector3(0f, 0f, x));

            if (!hasPreviousPanelPositions)
            {
                previousPanelPositions[i] = p;
                continue;
            }

            Vector3 vRel = (p - previousPanelPositions[i]) / Time.fixedDeltaTime;
            previousPanelPositions[i] = p;

            Vector3 normale = transform.right;
            float vN = Vector3.Dot(vRel, normale);
            Vector3 vT = vRel - vN * normale;

            float pressionDynamique = 0.5f * densiteEau * sPanel * coefTrainee;
            float fNormale = pressionDynamique * vN * Mathf.Abs(vN);
            Vector3 fTangente = pressionDynamique * vT.magnitude * vT;

            rb.AddForceAtPosition(-fNormale * normale - fTangente, p, ForceMode.Force);
        }

        hasPreviousPanelPositions = true;
    }
}