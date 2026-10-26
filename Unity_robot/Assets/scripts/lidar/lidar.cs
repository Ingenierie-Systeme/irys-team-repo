using System.Collections.Generic;
using UnityEngine;

using ROSConnection = Unity.Robotics.ROSTCPConnector.ROSConnection;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;

//simule le comportement d'un lidar en utilisant des raycasts 
// pour détecter les distances aux obstacles dans un environnement 3D
// les distances sont ensuites envoyées sous forme de messages PointCloud2 sur un topic ROS 2
// il faut associer un objet de type ParticleSystem pour visualiser les impacts des rayons du lidar dans la scène Unity

public class lidar : MonoBehaviour
{
    public int numberOfIncrements = 360;
    public int incrementAngle = 50; // nombre de rayons verticaux
    public float verticalFieldOfView = 50f; // angle vertical total, centré sur l'horizon
    public float maxRange = 100f;

    [Header("Affichage des impacts")]
    public ParticleSystem hitParticleSystem;

    [Header("Publication ROS 2")]
    public string pointCloudTopic = "/lidar/points";
    public string frameId = "lidar";
    public float publishPeriod = 0.1f;

    [Header("Détection")]
    public LayerMask detectionMask = ~0;
    public QueryTriggerInteraction triggerInteraction = QueryTriggerInteraction.Ignore;

    [HideInInspector]
    public static float[] distances;

    ROSConnection ros;
    float nextPublishTime;
    readonly List<Vector3> detectedPoints = new List<Vector3>();

    // Use this for initialization
    void Start()
    {
        distances = new float[numberOfIncrements*incrementAngle];
        //connexion ROS
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<PointCloud2Msg>(pointCloudTopic);

        if (hitParticleSystem != null)
        {
            var main = hitParticleSystem.main;
            main.maxParticles = numberOfIncrements * incrementAngle;
        }
    }

    // Update is called once per frame
    void FixedUpdate()
    {
        Vector3 fwd = new Vector3(0, 0, 1);
        Vector3 dir;
        RaycastHit hit;
        int indx = 0;

        // Efface les anciens impacts avant d'afficher le nouveau balayage.
        if (hitParticleSystem != null)
            hitParticleSystem.Clear();

        detectedPoints.Clear();

        for (int incr = 0; incr < numberOfIncrements; incr++)
        {
            for (int i = 0; i < incrementAngle; i++)
            {
                indx = incr * incrementAngle + i;
                // Répartit les rayons sur n degrés, centrés sur l'horizon.
                float verticalAngle = incrementAngle == 1
                    ? 0f
                    : Mathf.Lerp(-verticalFieldOfView / 2f,
                        verticalFieldOfView / 2f,
                        i / (float)(incrementAngle - 1));

                float horizontalAngle = incr * 360f / numberOfIncrements;

                // En Unity, une rotation X positive incline Vector3.forward vers le bas.
                // Le signe moins permet donc de considérer un angle vertical positif vers le haut.
                Vector3 localDirection = Quaternion.Euler(
                    -verticalAngle, horizontalAngle, 0f) * fwd;
                dir = transform.TransformDirection(localDirection).normalized;


                if (Physics.Raycast(transform.position,dir,out hit,maxRange,detectionMask,triggerInteraction))
                {
                    distances[indx] = (float)hit.distance;
                    detectedPoints.Add(transform.InverseTransformPoint(hit.point));

                    if (hitParticleSystem != null)
                    {
                        ParticleSystem.EmitParams emitParams = new ParticleSystem.EmitParams
                        {
                            // Emit utilise les coordonnées locales du ParticleSystem.
                            position = hitParticleSystem.transform.InverseTransformPoint(hit.point)
                        };

                        hitParticleSystem.Emit(emitParams, 1);
                    }
                }
                else
                {
                    distances[indx] = maxRange;
                }
            }
        }

        if (Time.time >= nextPublishTime)
        {
            PublishPointCloud();
            nextPublishTime = Time.time + Mathf.Max(0.01f, publishPeriod);
        }
    }
    //publication des messages
    void PublishPointCloud()
    {
        int pointStep = 12; // trois float32 : x, y et z
        byte[] data = new byte[detectedPoints.Count * pointStep];

        for (int i = 0; i < detectedPoints.Count; i++)
        {
            Vector3 point = detectedPoints[i];

            // Conversion Unity (x droite, y haut, z avant) vers ROS (x avant,
            // y gauche, z haut), tout en restant dans le repère local du lidar.
            WriteFloat(data, i * pointStep, point.z);
            WriteFloat(data, i * pointStep + 4, -point.x);
            WriteFloat(data, i * pointStep + 8, point.y);
        }

        PointCloud2Msg message = new PointCloud2Msg
        {
            header = new HeaderMsg { frame_id = frameId },
            height = 1,
            width = (uint)detectedPoints.Count,
            fields = new[]
            {
                new PointFieldMsg { name = "x", offset = 0, datatype = 7, count = 1 },
                new PointFieldMsg { name = "y", offset = 4, datatype = 7, count = 1 },
                new PointFieldMsg { name = "z", offset = 8, datatype = 7, count = 1 }
            },
            is_bigendian = false,
            point_step = (uint)pointStep,
            row_step = (uint)data.Length,
            data = data,
            is_dense = true
        };

        ros.Publish(pointCloudTopic, message);
    }

    static void WriteFloat(byte[] data, int offset, float value)
    {
        byte[] bytes = System.BitConverter.GetBytes(value);
        System.Buffer.BlockCopy(bytes, 0, data, offset, sizeof(float));
    }
}