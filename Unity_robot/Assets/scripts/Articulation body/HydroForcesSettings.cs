using UnityEngine;
using UnityEngine.Serialization;

[CreateAssetMenu(fileName = "HydroForcesSettings", menuName = "Robot/Hydro Forces Settings")]
public class HydroForcesSettings : ScriptableObject
{
    [Min(1)]
    public int nPanels = 10;

    [Min(0f)]
    public float densiteEau = 1000f;

    [Min(0f)]
    public float coefTraineeNormale = 50f;

    [Min(0f)]
    public float coefTraineeTangentielle = 0.05f;

    [Header("Immersion")]
    public float waterLevel;
    public Vector3 waterVelocity;
    [Min(1)]
    public int echantillonsImmersion = 6;

    [Header("Portance optionnelle")]
    public bool utiliserPortance;
    [Min(0f)]
    public float facteurPortance = 1f;
    [Range(1f, 45f)]
    public float angleDecrochage = 15f;

    [Header("Articulation")]
    [Min(0f)]
    [FormerlySerializedAs("articulationstiffness")]
    public float articulationStiffness = 20f;
    [Min(0f)]
    [FormerlySerializedAs("articulationdamping")]
    public float articulationDamping = 10f;
}
