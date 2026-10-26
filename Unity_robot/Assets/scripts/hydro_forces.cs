using UnityEngine;

[RequireComponent(typeof(Rigidbody))]
public class Nageoire : MonoBehaviour
{
    public int nPanels = 5;
    public float longueur = 0.20f;   // 20 cm
    public float hauteur = 0.15f;   // 15 cm
    public float densiteEau = 1000f;
    public float coefTrainee = 1.15f;

    private Rigidbody rb;

    void Awake() => rb = GetComponentInParent<Rigidbody>();

    void FixedUpdate()
    {
        float sPanel = longueur * hauteur / nPanels;

        for (int i = 0; i < nPanels; i++)
        {
            // position du centre du panel i le long de l'axe Z local (à adapter)
            float x = longueur * (i + 0.5f) / nPanels;
            Vector3 p = transform.TransformPoint(new Vector3(0f, 0f, x));

            Vector3 vRel = rb.GetPointVelocity(p);
            Vector3 normale = transform.right;
            float vN = Vector3.Dot(vRel, normale);
            Vector3 vT = vRel - vN * normale;

            // angle d'attaque → portance avec décrochage
            float alpha = Mathf.Deg2Rad * Vector3.Angle(vRel, normale);
            float CL = 2f * Mathf.PI * alpha * (alpha < 0.26f ? 1f : Mathf.Clamp01(0.16f / alpha));

            float fNormale = 0.5f * densiteEau * sPanel * CL * vN * Mathf.Abs(vN);
            float fTrainee = 0.5f * densiteEau * sPanel * coefTrainee * vT.magnitude * vT.magnitude;

            rb.AddForceAtPosition(-fNormale * normale - fTrainee * vT.normalized, p);
        }
    }
}