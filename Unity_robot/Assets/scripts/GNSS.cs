using System;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using RosMessageTypes.BuiltinInterfaces;


/// Simule un GNSS : publie la position (latitude/longitude/altitude) sur
/// sensor_msgs/msg/NavSatFix (topic /gnss/fix) via ROS-TCP-Connector.
///
/// La scène est supposée représenter une zone locale autour d'un point
/// géographique de référence (originLat/originLon) : la position Unity
/// (mètres) est convertie en décalage lat/lon autour de cette origine.
/// Placez ce composant sur un GameObject enfant de votre robot.

public class GNSS : MonoBehaviour
{
    [Header("Topic ROS")]
    public string topicName = "/gnss/fix";
    public string frameId = "gnss_link";

    [Header("Origine géographique de la scène")]
    [Tooltip("Latitude du point (0,0) de la scène, en degrés")]
    public double originLat = 48.39;   // Brest par défaut
    [Tooltip("Longitude du point (0,0) de la scène, en degrés")]
    public double originLon = -4.48;
    [Tooltip("Altitude du point (0,0) de la scène, en mètres")]
    public double originAlt = 0.0;

    [Header("Options")]
    [Tooltip("Période de publication en secondes (ex: 0.2 = 5 Hz, typique GNSS)")]
    public float publishPeriod = 0.2f;
    [Tooltip("Ajouter du bruit gaussien sur la position (mètres, écart-type)")]
    public float noiseStdDev = 0f;
    [Tooltip("perte de connexion en profondeur")]
    public float p_lost = 5f;

    private ROSConnection ros;
    private Vector3 position;
    private float lastPublishTime;

    // Rayon terrestre moyen (m) et facteur de conversion degrés -> radians
    const double EarthRadius = 6371000.0;
    const double DegToRad = Math.PI / 180.0;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<NavSatFixMsg>(topicName);
    }

    void Update()
    {
        // Position du robot via le parent
        position = transform.parent != null ? transform.parent.position : transform.position;

        if (Time.time - lastPublishTime >= publishPeriod)
        {
            lastPublishTime = Time.time;
            PublishFix();
        }
    }

    (double lat, double lon) MetersToLatLon(Vector3 unityPos)
    {
        // Unity (gaucher, Y-up) -> Nord-Est local (ROS convention ENU) :
        //   Nord = +Z Unity, Est = +X Unity
        double north = unityPos.z;
        double east = unityPos.x;
        double altitude = unityPos.y;

        // Conversion locale -> géographique.
        //la longitude se resserre selon cos(latitude).
        double lat = originLat + (north / EarthRadius) / DegToRad;
        double lon = originLon + (east / (EarthRadius * Math.Cos(originLat * DegToRad))) / DegToRad;

        return (lat, lon);
    }

    double GetAltitude(Vector3 unityPos)
    {
        return originAlt + unityPos.y;
    }

    Vector3 AddNoise(Vector3 pos)
    {
        if (noiseStdDev <= 0f) return pos;
        // Box-Muller : bruit gaussien centré
        double Gauss()
        {
            double u1 = UnityEngine.Random.value;
            double u2 = UnityEngine.Random.value;
            if (u1 < 1e-10) u1 = 1e-10;
            return Math.Sqrt(-2.0 * Math.Log(u1)) * Math.Cos(2.0 * Math.PI * u2);
        }
        return new Vector3(
            pos.x + (float)(Gauss() * noiseStdDev),
            pos.y + (float)(Gauss() * noiseStdDev),
            pos.z + (float)(Gauss() * noiseStdDev)
        );
    }

    NavSatFixMsg BuildMsg(Vector3 pos)
    {
        var (lat, lon) = MetersToLatLon(pos);
        double alt = GetAltitude(pos);

        var stamp = new TimeMsg
        {
            sec = (int)DateTimeOffset.UtcNow.ToUnixTimeSeconds(),
            nanosec = (uint)(DateTimeOffset.UtcNow.Ticks % TimeSpan.TicksPerSecond * 100)
        };

        return new NavSatFixMsg
        {
            header = new HeaderMsg { stamp = stamp, frame_id = frameId },
            status = new NavSatStatusMsg
            {
                status = (sbyte)NavSatStatusMsg.STATUS_SBAS_FIX, // = 2, fix valide
                service = (ushort)NavSatStatusMsg.SERVICE_GPS
            },
            latitude = lat,
            longitude = lon,
            altitude = alt,
            // Covariance diagonale : bruit (m²) sur lat/lon/alt.
            // Ici 1 m² en XY et 4 m² en Z, ordre du message :
            // [lat, lon, alt] en ligne/colonne 0,1,2.
            position_covariance = new double[]
            {
                noiseStdDev > 0 ? noiseStdDev * noiseStdDev : 1.0, 0, 0,
                0, noiseStdDev > 0 ? noiseStdDev * noiseStdDev : 1.0, 0,
                0, 0, 4.0
            },
            position_covariance_type = (byte)NavSatFixMsg.COVARIANCE_TYPE_DIAGONAL_KNOWN
        };
    }

    void PublishFix()
    {
        if (GetAltitude(position) > -p_lost)
        {
            ros.Publish(topicName, BuildMsg(AddNoise(position)));
        }
    }
}