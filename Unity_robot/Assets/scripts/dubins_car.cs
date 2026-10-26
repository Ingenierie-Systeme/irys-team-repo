using UnityEngine;
using UnityEngine.InputSystem;

// Contrôleur simple d'une voiture de Dubins sans roues ni colliders.
// Le script met à jour uniquement le transform de l'objet (plan X,Z).
public class dubins_car : MonoBehaviour
{
    [Header("Paramètres du modèle")]
    public float v = 2f; // vitesse max (m/s)
    public float R = 2f; // rayon de courbure minimal
    public bool forwardOnly = true; // si vrai, pas de marche arrière

    [Header("Commande (peut être pilotée par clavier)")]
    public float u1 = 1f; // vitesse commandée (m/s)
    public float u2 = 0f; // commande de rotation (rad/s)
    public bool useKeyboard = true; // activer contrôles clavier

    // état interne
    Vector2 X; // position (x, y) -> mappée sur (x, z) Unity
    float phi; // orientation en radians

    void Start()
    {
        // initialiser l'état à partir du transform
        Vector3 p = transform.position;
        X = new Vector2(p.z, p.x);
        phi = transform.eulerAngles.y * Mathf.Deg2Rad;
    }

    void Update()
    {
        float dt = Time.deltaTime;

        if (useKeyboard)
        {
            // up = avancer, down = arrêter / reculer (si forwardOnly=false)
            if (Input.GetKey(KeyCode.UpArrow)) u1 = v;
            else if (Input.GetKey(KeyCode.DownArrow)) u1 = forwardOnly ? 0f : -v;
            else u1 = 0f;

            // left/right = tourner (entrée binaire simple)
            if (Input.GetKey(KeyCode.LeftArrow)) u2 = v / R; // tourner à gauche
            else if (Input.GetKey(KeyCode.RightArrow)) u2 = -v / R; // tourner à droite
            else u2 = 0f;
        }

        // contraintes (sécurité)
        if (forwardOnly) u1 = Mathf.Clamp(u1, 0f, v);
        else u1 = Mathf.Clamp(u1, -v, v);

        u2 = Mathf.Clamp(u2, -v / R, v / R);

        // intégration simple des équations cinématiques de Dubins
        phi += u2 * dt;
        X += new Vector2(u1 * Mathf.Cos(phi), u1 * Mathf.Sin(phi)) * dt;

        // appliquer au transform (plan X-Z)
        transform.position = new Vector3(X.y, transform.position.y, X.x);
        transform.rotation = Quaternion.Euler(0f, phi * Mathf.Rad2Deg, 0f);
    }
}
