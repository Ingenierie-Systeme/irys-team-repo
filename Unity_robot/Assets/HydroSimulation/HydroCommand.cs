using UnityEngine;
namespace Irys.Hydro
{
    public struct HydroSetpoint { public float frequencyHz, amplitudeDeg, meanDeg; }
    // Stateful f(u,v). An elliptical A/f arc, not a calibrated power measurement.
    public sealed class HydroCommand
    {
        float filteredU, mean;
        public HydroSetpoint Current { get; private set; }
        public static HydroSetpoint Map(float intensity, float meanDeg)
        {
            float theta = Mathf.Clamp01(intensity) * Mathf.PI * .5f;
            float f = RobotHydroDefinition.MaxFrequencyHz * (1f - Mathf.Cos(theta));
            float a = RobotHydroDefinition.MaxAmplitudeDeg * Mathf.Sin(theta);
            meanDeg = Mathf.Clamp(meanDeg, -RobotHydroDefinition.MaxMeanDeg, RobotHydroDefinition.MaxMeanDeg);
            float meanBudget = 2f * RobotHydroDefinition.MaxMeanDeg / RobotHydroDefinition.MeanFilterSeconds;
            if (f > 0f) a = Mathf.Min(a, (RobotHydroDefinition.CommandSpeedDeg - meanBudget) / (2f * Mathf.PI * f));
            a = Mathf.Min(a, RobotHydroDefinition.TravelDeg - Mathf.Abs(meanDeg));
            return new HydroSetpoint { frequencyHz = f, amplitudeDeg = Mathf.Max(0f, a), meanDeg = meanDeg };
        }
        public HydroSetpoint Step(float u, float v, float dt)
        {
            if (!float.IsFinite(u) || !float.IsFinite(v) || !float.IsFinite(dt) || dt <= 0f)
                throw new System.ArgumentOutOfRangeException("Finite inputs and positive dt required.");
            filteredU += (Mathf.Clamp01(u) - filteredU) * (1f - Mathf.Exp(-dt / RobotHydroDefinition.PropulsionFilterSeconds));
            mean += (Mathf.Clamp(v, -1f, 1f) * RobotHydroDefinition.MaxMeanDeg - mean) * (1f - Mathf.Exp(-dt / RobotHydroDefinition.MeanFilterSeconds));
            return Current = Map(filteredU, mean);
        }
    }
}
