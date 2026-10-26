using System;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using RosMessageTypes.BuiltinInterfaces;


/// Simule une IMU : publie orientation, vitesses angulaires et accélération
/// sur sensor_msgs/msg/Imu (topic /imu/data) via ROS-TCP-Connector.
/// Placez ce composant sur un GameObject avec un Rigidbody.

public class IMU : MonoBehaviour
{
    [Header("Topic ROS")]
    public string topicName = "/imu/data";
    public string frameId = "imu_link";
    [Tooltip("Période de publication en secondes (ex: 0.02 = 50 Hz)")]
    public float publishPeriod = 0.02f;

    [Header("Références")]
    public Rigidbody rb;

    [Header("Options")]
    [Tooltip("Ajouter la gravité à l'accélération mesurée (une vraie IMU la mesure)")]
    public bool includeGravity = true;

    private ROSConnection ros;

    // État interne
    private Quaternion orientation;      // orientation dans le repère ROS
    private Vector3 linSpeed;           // vitesse linéaire (repère monde Unity)
    private Vector3 prevLinSpeed;       // vitesse à l'itération physique précédente
    private Vector3 angSpeed;           // vitesse angulaire (repère corps, rad/s)
    private Vector3 linAcc;             // accélération linéaire
    private float lastPublishTime;
    private bool hasPrevSpeed;          // évite un pic d'accélération au 1er frame

    // Conversion Unity (gaucher, Y-up) -> ROS (droitier, Z-up)
    static readonly Quaternion UnityToRosRotation =
        Quaternion.Euler(0f, 180f, 0f) * Quaternion.Euler(-90f, 0f, 0f);

    void Start()
    {
        if (rb == null)
            rb = GetComponent<Rigidbody>();
        if (rb == null)
        {
            Debug.LogError("IMU nécessite un Rigidbody.");
            enabled = false;
            return;
        }

        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<ImuMsg>(topicName);
    }

    // Toute la physique dans FixedUpdate : les vitesses du Rigidbody
    // ne sont fiables qu'après le pas de simulation physique.
    void FixedUpdate()
    {
        float dt = Time.fixedDeltaTime;

        // Vitesse angulaire : rb.angularVelocity est déjà en rad/s dans le
        // repère du corps, mais repère Unity -> convertir vers ROS.
        angSpeed = UnityToRos(rb.angularVelocity);

        // Accélération linéaire : dérivée de la vitesse.
        // Une IMU mesure l'accélération dans SON repère (body frame),
        // on ré-exprime donc l'accélération monde dans le repère du corps.
        linSpeed = rb.linearVelocity;
        if (hasPrevSpeed)
        {
            Vector3 accWorld = (linSpeed - prevLinSpeed) / dt;
            if (includeGravity)
                accWorld += Physics.gravity;
            // Rotation monde -> corps (inverse de la rotation du corps)
            linAcc = UnityToRos(Quaternion.Inverse(transform.rotation) * accWorld);
        }
        prevLinSpeed = linSpeed;
        hasPrevSpeed = true;

        // Orientation : rotation du corps, convertie vers le repère ROS.
        orientation = UnityToRosRotation * transform.parent.rotation;

        // Publication à fréquence fixe, indépendante du FixedUpdate
        if (Time.time - lastPublishTime >= publishPeriod)
        {
            lastPublishTime = Time.time;
            PublishImu();
        }
    }

    static Vector3 UnityToRos(Vector3 v)
    {
        // Ré-exprime un vecteur du repère Unity vers le repère ROS
        // (même transformation que UnityToRosRotation) :
        // Unity (x,y,z) -> ROS (z, -x, y)
        return new Vector3(v.z, -v.x, v.y);
    }

    ImuMsg BuildMsg()
    {
        var stamp = new TimeMsg
        {
            sec = (int)DateTimeOffset.UtcNow.ToUnixTimeSeconds(),
            nanosec = (uint)(DateTimeOffset.UtcNow.Ticks % TimeSpan.TicksPerSecond * 100)
        };

        return new ImuMsg
        {
            header = new HeaderMsg { stamp = stamp, frame_id = frameId },
            orientation = new RosMessageTypes.Geometry.QuaternionMsg
            {
                x = orientation.x,
                y = orientation.y,
                z = orientation.z,
                w = orientation.w
            },
            angular_velocity = new RosMessageTypes.Geometry.Vector3Msg(angSpeed.x, angSpeed.y, angSpeed.z),
            linear_acceleration = new RosMessageTypes.Geometry.Vector3Msg(linAcc.x, linAcc.y, linAcc.z),
            // Covariances : 0 = inconnue ; mettre une diagonale réaliste
            // (ex: 0.02 orientation, 0.004 vitesses/accélérations) si un
            // filtre type robot_localization les utilise.
            orientation_covariance = new double[9],
            angular_velocity_covariance = new double[9],
            linear_acceleration_covariance = new double[9]
        };
    }

    void PublishImu()
    {
        ros.Publish(topicName, BuildMsg());
    }
}