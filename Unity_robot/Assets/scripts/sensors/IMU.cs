using System;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using RosMessageTypes.BuiltinInterfaces;


/// Simule une IMU : publie orientation, vitesses angulaires et accélération
/// sur sensor_msgs/msg/Imu (topic /imu/data) via ROS-TCP-Connector.
/// Placez ce composant sur un GameObject enfant d'une ArticulationBody.

public class IMU : MonoBehaviour
{
    [Header("Topic ROS")]
    public string topicName = "/input/imu";
    public string frameId = "imu_link";
    [Tooltip("Période de publication en secondes (ex: 0.02 = 50 Hz)")]
    public float publishPeriod = 0.02f;

    [Header("Références")]
    public ArticulationBody articulationBody;

    [Header("Options")]
    [Tooltip("Inclure la gravité ressentie par une IMU au repos")]
    public bool includeGravity = true;

    private ROSConnection ros;

    // État interne
    private Quaternion orientation;      // orientation dans le repère ROS
    private Vector3 pointVelocity;      // vitesse au point capteur (repère monde)
    private Vector3 prevPointVelocity;  // vitesse au point capteur précédente
    private Vector3 angSpeed;           // vitesse angulaire (repère capteur, rad/s)
    private Vector3 linAcc;             // accélération linéaire
    private float lastPublishTime;
    private bool hasPrevSpeed;          // évite un pic d'accélération au 1er frame

    void Start()
    {
        if (articulationBody == null)
            articulationBody = GetComponentInParent<ArticulationBody>();
        if (articulationBody == null)
        {
            Debug.LogError(
                "IMU nécessite une ArticulationBody sur elle-même ou un parent.",
                this);
            enabled = false;
            return;
        }

        if (publishPeriod <= 0f)
        {
            Debug.LogError("La période de publication IMU doit être positive.", this);
            enabled = false;
            return;
        }

        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<ImuMsg>(topicName);
    }

    // Toute la physique est lue dans FixedUpdate après le pas de simulation.
    void FixedUpdate()
    {
        float dt = Time.fixedDeltaTime;

        // ArticulationBody fournit les vitesses dans le repère monde Unity.
        Quaternion sensorRotation = transform.rotation;
        Vector3 angularVelocitySensor =
            Quaternion.Inverse(sensorRotation) * articulationBody.angularVelocity;
        angSpeed = UnityToRos(angularVelocitySensor);

        // Échantillonner la vitesse au capteur inclut l'effet de la rotation
        // si celui-ci est décalé du centre de l'ArticulationBody.
        pointVelocity = articulationBody.GetPointVelocity(transform.position);
        if (hasPrevSpeed)
        {
            Vector3 accWorld = (pointVelocity - prevPointVelocity) / dt;
            if (includeGravity)
                accWorld -= Physics.gravity;
            Vector3 accelerationSensor =
                Quaternion.Inverse(sensorRotation) * accWorld;
            linAcc = UnityToRos(accelerationSensor);
        }
        else
        {
            linAcc = Vector3.zero;
        }
        prevPointVelocity = pointVelocity;
        hasPrevSpeed = true;

        // Orientation du capteur, convertie vers le repère ROS.
        orientation = UnityToRosOrientation(sensorRotation);

        // Publication à fréquence fixe, indépendante du FixedUpdate
        if (Time.fixedTime - lastPublishTime >= publishPeriod)
        {
            lastPublishTime = Time.fixedTime;
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

    static Quaternion UnityToRosOrientation(Quaternion rotation)
    {
        // Changement de base gaucher/droitier, cohérent avec UnityToRos.
        return new Quaternion(-rotation.z, rotation.x, -rotation.y, rotation.w);
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