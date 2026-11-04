using UnityEngine;
namespace Irys.Hydro
{
    // Fixed SI reference. Masses and damping are engineering assumptions, not identified values.
    public static class RobotHydroDefinition
    {
        public const float TailLength = .350f, TailHeight = .070f, TailThickness = .001f;
        public const float PvcDensity = 1400f, TestLength = .30f, TestDeflection = .05f;
        // Uniform own weight on a cantilever: delta=q L^4/(8 EI), tested thickness=1 mm.
        public static float YoungModulus => 1.5f * PvcDensity * 9.81f * Mathf.Pow(TestLength, 4) / (1e-6f * TestDeflection);
        public static float BendingRigidity => YoungModulus * TailHeight * Mathf.Pow(TailThickness, 3) / 12f;
        public const int Sections = 6;
        public const float HullLength = .400f, HullRadius = .055f, HullMass = 3.25f, ServoMass = .066f;
        public const float WaterDensity = 1000f, WaterLevel = 0f;
        // D955TW at 7.4 V. Assembly travel is provisional, not a manufacturer specification.
        public const float StallTorque = 2.844f, NoLoadSpeedDeg = 500f, TravelDeg = 60f;
        public const float MaxAmplitudeDeg = 28f, MaxFrequencyHz = 1.2f, MaxMeanDeg = 25f;
        public const float CommandSpeedDeg = 300f, CommandAccelerationDeg = 1800f;
        public const float MeanFilterSeconds = .65f, PropulsionFilterSeconds = .8f, StepSeconds = .0025f;
    }
}
