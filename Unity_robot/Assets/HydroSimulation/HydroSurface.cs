using UnityEngine;
namespace Irys.Hydro
{
    // Six equivalent directional surfaces. Mesh geometry is read once at initialization;
    // the physics step uses only areas, centers and a smooth immersion fraction.
    public sealed class HydroSurface
    {
        readonly Transform frame;
        readonly ArticulationBody body;
        readonly float cd, cf, volume;
        readonly Bounds bounds;
        readonly Vector3[] centers = new Vector3[6];
        readonly float[] areas = new float[6];
        static readonly Vector3[] normals = { Vector3.right, Vector3.left, Vector3.up, Vector3.down, Vector3.forward, Vector3.back };

        public HydroSurface(Mesh mesh, Transform frame, ArticulationBody body, float cd = 1.8f, float cf = .008f, bool buoyant = false)
        {
            this.frame = frame; this.body = body; this.cd = cd; this.cf = cf;
            bounds = mesh.bounds;
            // Project triangle areas onto the three local axes, once. This keeps the CAO
            // silhouette areas without evaluating individual triangles during simulation.
            var vertices = mesh.vertices; var triangles = mesh.triangles;
            float signedVolume = 0f;
            for (int i = 0; i < triangles.Length; i += 3)
            {
                Vector3 a = vertices[triangles[i]], b = vertices[triangles[i+1]], c = vertices[triangles[i+2]];
                Vector3 cross = Vector3.Cross(b-a, c-a) * .5f;
                Vector3 center = (a+b+c)/3f;
                signedVolume += Vector3.Dot(a, Vector3.Cross(b,c))/6f;
                for (int axis = 0; axis < 3; axis++)
                {
                    int face = 2*axis + (cross[axis] < 0f ? 1 : 0);
                    float area = Mathf.Abs(cross[axis]);
                    areas[face] += area; centers[face] += area*center;
                }
            }
            for (int i = 0; i < 6; i++)
                if (areas[i] > 0f) centers[i] /= areas[i];
            volume = buoyant ? Mathf.Abs(signedVolume) : 0f;
        }

        public HydroSurface(Bounds bounds, Transform frame, ArticulationBody body)
        {
            this.bounds = bounds; this.frame = frame; this.body = body; cd = 1.8f; cf = .008f;
            for (int i = 0; i < 6; i++)
            {
                int axis = i/2;
                centers[i] = bounds.center + Vector3.Scale(normals[i], bounds.extents);
                areas[i] = bounds.size[(axis+1)%3]*bounds.size[(axis+2)%3];
            }
        }

        public void Step(Vector3 current)
        {
            Matrix4x4 matrix = frame.localToWorldMatrix;
            Matrix4x4 normalMatrix = matrix.inverse.transpose;
            float scaleVolume = Mathf.Abs(matrix.determinant);
            Vector3 size = bounds.size;
            float height = Mathf.Max(1e-5f, Mathf.Abs(matrix.MultiplyVector(Vector3.right*size.x).y)
                + Mathf.Abs(matrix.MultiplyVector(Vector3.up*size.y).y)
                + Mathf.Abs(matrix.MultiplyVector(Vector3.forward*size.z).y));
            Vector3 center = matrix.MultiplyPoint3x4(bounds.center);
            float immersed = Mathf.Clamp01(.5f + (RobotHydroDefinition.WaterLevel-center.y)/height);
            if (immersed <= 0f) return;
            // A single partial-immersion approximation for each element, without clipping.
            Vector3 wetOffset = Vector3.down*(.5f*height*(1f-immersed));
            for (int i = 0; i < 6; i++)
            {
                if (areas[i] <= 0f) continue;
                Vector3 normal = normalMatrix.MultiplyVector(normals[i]);
                float area = areas[i]*scaleVolume*normal.magnitude*immersed;
                normal.Normalize();
                Vector3 p = matrix.MultiplyPoint3x4(centers[i]) + wetOffset;
                Vector3 velocity = body.linearVelocity + Vector3.Cross(body.angularVelocity,p-body.worldCenterOfMass) - current;
                float vn = Mathf.Max(0f,Vector3.Dot(velocity,normal));
                Vector3 tangent = velocity - Vector3.Dot(velocity,normal)*normal;
                Vector3 force = -.5f*RobotHydroDefinition.WaterDensity*area*
                    (cd*vn*vn*normal + cf*tangent.magnitude*tangent);
                body.AddForceAtPosition(force,p);
            }
            if (volume > 0f)
                body.AddForceAtPosition(Vector3.up*(RobotHydroDefinition.WaterDensity*9.81f*volume*scaleVolume*immersed),center+wetOffset);
        }
    }
}
