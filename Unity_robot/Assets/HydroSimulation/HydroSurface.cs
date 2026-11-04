using UnityEngine;
namespace Irys.Hydro
{
    // Integrates pressure on the upstream face of a closed mesh; no double-sided drag.
    // Clips at the horizontal water plane. Velocity and force are evaluated in world SI units.
    public sealed class HydroSurface
    {
        readonly Vector3[] vertices;
        readonly Vector3[] world;
        readonly int[] triangles;
        readonly Transform frame;
        readonly ArticulationBody body;
        readonly Vector3[] clipped = new Vector3[4];
        readonly float cd, cf;
        readonly bool buoyant;
        float signedVolume;
        Vector3 volumeMoment, volumeOrigin;
        Vector3 linearVelocity, angularVelocity, bodyCenter;
        public Vector3 Force { get; private set; }
        public Vector3 Torque { get; private set; }
        public float Power { get; private set; }
        public float DisplacedVolume => buoyant ? Mathf.Abs(signedVolume) : 0f;
        public HydroSurface(Mesh mesh, Transform frame, ArticulationBody body, float cd = 1.8f, float cf = .008f, bool buoyant = false)
        {
            if(!mesh.isReadable)throw new System.InvalidOperationException("Enable Read/Write in the import settings of hydrodynamic mesh: "+mesh.name);
            vertices = mesh.vertices; world=new Vector3[vertices.Length]; triangles = mesh.triangles; this.frame = frame; this.body = body; this.cd = cd; this.cf = cf; this.buoyant = buoyant;
            float volume=0f;
            for(int i=0;i<triangles.Length;i+=3) volume+=Vector3.Dot(vertices[triangles[i]],Vector3.Cross(vertices[triangles[i+1]],vertices[triangles[i+2]]));
            if(volume<0f) for(int i=0;i<triangles.Length;i+=3) (triangles[i+1],triangles[i+2])=(triangles[i+2],triangles[i+1]);
        }
        public static Vector3 Drag(Vector3 velocity, Vector3 normal, float area, float cd, float cf)
        {
            float vn = Vector3.Dot(velocity, normal);
            Vector3 vt = velocity - vn * normal;
            return -.5f * RobotHydroDefinition.WaterDensity * area *
                (cd * Mathf.Max(0f, vn) * Mathf.Max(0f, vn) * normal + cf * vt.magnitude * vt);
        }
        public void Step(Vector3 current)
        {
            Force = Vector3.zero; Torque=Vector3.zero; Power = 0f;
            signedVolume=0f;volumeMoment=Vector3.zero;
            volumeOrigin=new Vector3(frame.position.x,RobotHydroDefinition.WaterLevel,frame.position.z);
            Matrix4x4 matrix=frame.localToWorldMatrix;
            for(int i=0;i<vertices.Length;i++) world[i]=matrix.MultiplyPoint3x4(vertices[i]);
            linearVelocity=body.linearVelocity;angularVelocity=body.angularVelocity;bodyCenter=body.worldCenterOfMass;
            for (int i = 0; i < triangles.Length; i += 3)
            {
                Vector3 a = world[triangles[i]], b = world[triangles[i + 1]], c = world[triangles[i + 2]];
                int count = 0;
                Edge(c, a, ref count); Edge(a, b, ref count); Edge(b, c, ref count);
                for (int j = 1; j + 1 < count; j++) Apply(clipped[0], clipped[j], clipped[j + 1], current);
            }
            // The missing cap lies in the plane through volumeOrigin and has zero tetrahedral volume.
            if(buoyant && Mathf.Abs(signedVolume)>1e-12f)
                body.AddForceAtPosition(Vector3.up*(Mathf.Abs(signedVolume)*RobotHydroDefinition.WaterDensity*9.81f),volumeMoment/signedVolume);
            body.AddForce(Force,ForceMode.Force);body.AddTorque(Torque,ForceMode.Force);
        }
        void Edge(Vector3 a, Vector3 b, ref int count)
        {
            bool insideA = a.y <= RobotHydroDefinition.WaterLevel, insideB = b.y <= RobotHydroDefinition.WaterLevel;
            if (insideA != insideB) clipped[count++] = Vector3.LerpUnclamped(a, b, (RobotHydroDefinition.WaterLevel - a.y) / (b.y - a.y));
            if (insideB) clipped[count++] = b;
        }
        void Apply(Vector3 a, Vector3 b, Vector3 c, Vector3 current)
        {
            Vector3 cross = Vector3.Cross(b - a, c - a);
            float twiceArea = cross.magnitude;
            if (twiceArea < 1e-10f) return;
            if(buoyant)
            {
                float volume=Vector3.Dot(a-volumeOrigin,Vector3.Cross(b-volumeOrigin,c-volumeOrigin))/6f;
                signedVolume+=volume;volumeMoment+=volume*(volumeOrigin+a+b+c)*.25f;
            }
            Vector3 p = (a + b + c) / 3f, velocity = linearVelocity+Vector3.Cross(angularVelocity,p-bodyCenter)-current;
            Vector3 force = Drag(velocity, cross / twiceArea, twiceArea * .5f, cd, cf);
            Force += force; Torque+=Vector3.Cross(p-bodyCenter,force); Power += Vector3.Dot(force, velocity);
        }
    }
}
