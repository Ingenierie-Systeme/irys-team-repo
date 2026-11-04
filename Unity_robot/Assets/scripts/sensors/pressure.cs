using System;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;

/// <summary>
/// Simule un capteur de pression hydrostatique et publie sensor_msgs/FluidPressure.
/// La pression retournée est absolue, en pascals.
/// </summary>
public class pressure : MonoBehaviour
{
    [Header("Topic ROS")]
    public string topicName = "/input/pressure";
    public string frameId = "pressure_link";
    [Min(0.01f)]
    public float publishPeriod = 0.1f;

    [Header("Environnement")]
    [Tooltip("Altitude Y de la surface de l'eau, en mètres (repère Unity).")]
    public float waterLevel;
    [Min(0f)]
    public float waterDensity = 1000f;
    [Min(0f)]
    public float atmosphericPressure = 101325f;
    [Min(0f)]
    public float variance;

    [Header("Mesure")]
    [SerializeField] private float profondeur;
    [SerializeField] private double pressionPa;

    private ROSConnection ros;
    private float lastPublishTime;

    public float Depth => profondeur;
    public double PressurePa => pressionPa;

    private void Start()
    {
        if (publishPeriod <= 0f || !IsFinite(publishPeriod) ||
            waterDensity <= 0f || !IsFinite(waterDensity) ||
            atmosphericPressure < 0f || !IsFinite(atmosphericPressure) ||
            variance < 0f || !IsFinite(variance) || !IsFinite(waterLevel))
        {
            Debug.LogError(
                "La période doit être positive, la densité strictement positive et les autres paramètres finis et non négatifs.",
                this);
            enabled = false;
            return;
        }

        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<FluidPressureMsg>(topicName);
    }

    private void FixedUpdate()
    {
        profondeur = Mathf.Max(0f, waterLevel - transform.position.y);
        pressionPa = atmosphericPressure +
                     waterDensity * Physics.gravity.magnitude * profondeur;

        if (Time.fixedTime - lastPublishTime < publishPeriod)
            return;

        lastPublishTime = Time.fixedTime;
        PublishPressure();
    }

    private void PublishPressure()
    {
        DateTimeOffset now = DateTimeOffset.UtcNow;
        var stamp = new TimeMsg
        {
            sec = (int)now.ToUnixTimeSeconds(),
            nanosec = (uint)(now.Ticks % TimeSpan.TicksPerSecond * 100)
        };

        var message = new FluidPressureMsg
        {
            header = new HeaderMsg { stamp = stamp, frame_id = frameId },
            fluid_pressure = pressionPa,
            variance = variance
        };

        ros.Publish(topicName, message);
    }

    private static bool IsFinite(float value)
    {
        return !float.IsNaN(value) && !float.IsInfinity(value);
    }
}