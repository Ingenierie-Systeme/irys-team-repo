using System.Collections.Generic;
using UnityEngine;
#if ENABLE_INPUT_SYSTEM
using UnityEngine.InputSystem;
#endif

namespace Irys.Hydro
{
    [DefaultExecutionOrder(-100)]
    public sealed class HydroRobot : MonoBehaviour
    {
        public ArticulationBody hull, servo;
        public ArticulationBody[] sections;
        public MeshFilter hullSurface;
        public MeshFilter[] fixedFins;
        public Material tailMaterial;
        [Range(0f, 1f)] public float u;
        [Range(-1f, 1f)] public float v;
        public bool keyboard = true, showHud = true;
        public Vector3 current;
        public HydroSetpoint Setpoint { get; private set; }
        public float ServoTarget { get; private set; }
        public float ServoAngle => servo.jointPosition.dofCount > 0 ? servo.jointPosition[0] * Mathf.Rad2Deg : 0f;
        public float TorqueLimit { get; private set; }
        public float DragPower { get; private set; }
        public float SubmergedVolume { get; private set; }
        public int Steps { get; private set; }
        public float TimeSimulated { get; private set; }
        readonly HydroCommand command = new HydroCommand();
        readonly List<HydroSurface> surfaces = new List<HydroSurface>();
        readonly List<Vector3> cells = new List<Vector3>();
        readonly List<Mesh> transientMeshes = new List<Mesh>();
        float cellVolume, phase, targetSpeed;
        Vector3[] skinPoints;
        Mesh skin;
        Transform skinFrame;
        const float Cell = .01f;
        bool initialized;
        void Awake() { Time.fixedDeltaTime=RobotHydroDefinition.StepSeconds; }

        void Start() { Initialize(); }
        public void Initialize()
        {
            if(initialized) return; initialized=true;
            surfaces.Add(new HydroSurface(hullSurface.sharedMesh, hullSurface.transform, hull, 1.0f));
            foreach (var fin in fixedFins) surfaces.Add(new HydroSurface(fin.sharedMesh, fin.transform, hull, buoyant: true));
            float ds = RobotHydroDefinition.TailLength / sections.Length;
            foreach (var section in sections)
            {
                GameObject panel = new GameObject("HydroPanel");
                panel.transform.SetParent(section.transform, false);
                panel.transform.localPosition = Vector3.back * ds * .5f;
                panel.transform.localScale = new Vector3(RobotHydroDefinition.TailThickness, RobotHydroDefinition.TailHeight, ds);
                Mesh mesh = BoxMesh(); transientMeshes.Add(mesh);
                surfaces.Add(new HydroSurface(mesh, panel.transform, section));
            }
            int nz = Mathf.RoundToInt(RobotHydroDefinition.HullLength / Cell);
            int nr = Mathf.CeilToInt(2f * RobotHydroDefinition.HullRadius / Cell);
            for (int z = 0; z < nz; z++) for (int x = 0; x < nr; x++) for (int y = 0; y < nr; y++)
            {
                Vector3 p = new Vector3((x + .5f - nr / 2f) * Cell, (y + .5f - nr / 2f) * Cell, (z + .5f - nz / 2f) * Cell);
                if (p.x * p.x + p.y * p.y <= RobotHydroDefinition.HullRadius * RobotHydroDefinition.HullRadius) cells.Add(p);
            }
            cellVolume = Mathf.PI * RobotHydroDefinition.HullRadius * RobotHydroDefinition.HullRadius * RobotHydroDefinition.HullLength / cells.Count;
            BuildSkin();
        }

        void Update()
        {
            if (!keyboard) return;
            float throttle = 0f, steering = 0f;
#if ENABLE_INPUT_SYSTEM
            var k = Keyboard.current;
            if (k == null) return;
            throttle = (k.upArrowKey.isPressed || k.wKey.isPressed ? 1f : 0f) - (k.downArrowKey.isPressed || k.sKey.isPressed ? 1f : 0f);
            steering = (k.leftArrowKey.isPressed || k.aKey.isPressed ? 1f : 0f) - (k.rightArrowKey.isPressed || k.dKey.isPressed ? 1f : 0f);
            if (k.spaceKey.wasPressedThisFrame) u = 0f;
#else
            throttle = (Input.GetKey(KeyCode.UpArrow) ? 1f : 0f) - (Input.GetKey(KeyCode.DownArrow) ? 1f : 0f);
            steering = (Input.GetKey(KeyCode.LeftArrow) ? 1f : 0f) - (Input.GetKey(KeyCode.RightArrow) ? 1f : 0f);
            if (Input.GetKeyDown(KeyCode.Space)) u = 0f;
#endif
            u = Mathf.Clamp01(u + throttle * .35f * Time.deltaTime);
            // Keyboard is momentary; sliders/external input remain available when keyboard=false.
            v = steering;
        }

        void FixedUpdate() { Step(Time.fixedDeltaTime); }
        public void Step(float dt)
        {
            Initialize();
            Setpoint = command.Step(u, v, dt);
            phase = Mathf.Repeat(phase + 2f * Mathf.PI * Setpoint.frequencyHz * dt, 2f * Mathf.PI);
            float desired = Setpoint.meanDeg + Setpoint.amplitudeDeg * Mathf.Sin(phase);
            float speed = Mathf.Clamp(30f * (desired - ServoTarget), -RobotHydroDefinition.CommandSpeedDeg, RobotHydroDefinition.CommandSpeedDeg);
            targetSpeed = Mathf.MoveTowards(targetSpeed, speed, RobotHydroDefinition.CommandAccelerationDeg * dt);
            float delta = targetSpeed * dt;
            ServoTarget = Mathf.Clamp(ServoTarget + delta, -RobotHydroDefinition.TravelDeg, RobotHydroDefinition.TravelDeg);
            float actualSpeed = servo.jointVelocity[0] * Mathf.Rad2Deg;
            var drive = servo.xDrive;
            // Linear torque-speed approximation, with a full stall-torque budget while braking.
            float requestedTorque = drive.stiffness * (ServoTarget*Mathf.Deg2Rad-servo.jointPosition[0])
                + drive.damping * (targetSpeed*Mathf.Deg2Rad-servo.jointVelocity[0]);
            bool motoring = actualSpeed * requestedTorque > 0f;
            TorqueLimit = RobotHydroDefinition.StallTorque * (motoring ? Mathf.Clamp01(1f - Mathf.Abs(actualSpeed) / RobotHydroDefinition.NoLoadSpeedDeg) : 1f);
            drive.target = ServoTarget; drive.targetVelocity = targetSpeed; drive.forceLimit = TorqueLimit;
            servo.xDrive = drive;
            Buoyancy();
            DragPower = 0f;
            foreach (var surface in surfaces) { surface.Step(current); DragPower += surface.Power; }
            float ds = RobotHydroDefinition.TailLength / sections.Length;
            foreach (var section in sections)
            {
                Vector3 p = section.transform.TransformPoint(Vector3.back * ds * .5f);
                float fraction = Mathf.Clamp01(.5f + (RobotHydroDefinition.WaterLevel - p.y) / RobotHydroDefinition.TailHeight);
                section.AddForceAtPosition(Vector3.up * (fraction * RobotHydroDefinition.WaterDensity * 9.81f * RobotHydroDefinition.TailHeight * ds * RobotHydroDefinition.TailThickness), p);
            }
            Steps++; TimeSimulated += dt;
        }

        void Buoyancy()
        {
            SubmergedVolume = 0f; Vector3 moment = Vector3.zero;
            Vector3 up = hull.transform.InverseTransformDirection(Vector3.up);
            Matrix4x4 matrix=hull.transform.localToWorldMatrix;
            float height = Cell * (Mathf.Abs(up.x) + Mathf.Abs(up.y) + Mathf.Abs(up.z));
            foreach (Vector3 cell in cells)
            {
                Vector3 p = matrix.MultiplyPoint3x4(cell);
                float fraction = Mathf.Clamp01(.5f + (RobotHydroDefinition.WaterLevel - p.y) / height);
                float volume = fraction * cellVolume;
                // Shift the center to the centroid of the submerged vertical slice.
                Vector3 center = p - Vector3.up * (.5f * height * (1f - fraction));
                SubmergedVolume += volume; moment += center * volume;
            }
            if (SubmergedVolume > 0f) hull.AddForceAtPosition(Vector3.up * (RobotHydroDefinition.WaterDensity * 9.81f * SubmergedVolume), moment / SubmergedVolume);
        }

        void BuildSkin()
        {
            skinFrame = new GameObject("Palme_PVC_continue_350x70x1mm").transform;
            skinFrame.SetParent(transform, false);
            skin = new Mesh { name = "PVC continuous skin" }; transientMeshes.Add(skin);
            skinFrame.gameObject.AddComponent<MeshFilter>().sharedMesh = skin;
            skinFrame.gameObject.AddComponent<MeshRenderer>().sharedMaterial = tailMaterial;
            int rows = sections.Length * 4 + 1;
            skinPoints = new Vector3[rows * 4];
            var indices = new List<int>();
            for (int row = 0; row < rows - 1; row++) for (int side = 0; side < 4; side++)
            {
                int a = row * 4 + side, b = row * 4 + (side + 1) % 4, c = a + 4, d = b + 4;
                indices.AddRange(new[] { a, c, b, b, c, d });
            }
            indices.AddRange(new[] { 0, 1, 2, 0, 2, 3, (rows-1)*4, (rows-1)*4+2, (rows-1)*4+1, (rows-1)*4, (rows-1)*4+3, (rows-1)*4+2 });
            skin.vertices = skinPoints; skin.triangles = indices.ToArray();
            UpdateSkin();
        }
        Vector3 Boundary(int i)
        {
            if (i < 0) return 2f * sections[0].transform.position - Boundary(1);
            if (i < sections.Length) return sections[i].transform.position;
            Vector3 tip = sections[sections.Length - 1].transform.TransformPoint(Vector3.back * (RobotHydroDefinition.TailLength / sections.Length));
            return i == sections.Length ? tip : 2f * tip - Boundary(sections.Length - 1);
        }
        void LateUpdate() { if (skin != null) UpdateSkin(); }
        public void RefreshVisual() { if(skin!=null) UpdateSkin(); }
        void UpdateSkin()
        {
            int rows = skinPoints.Length / 4;
            for (int row = 0; row < rows; row++)
            {
                float s = (float)row / (rows - 1) * sections.Length;
                int i = Mathf.Min((int)s, sections.Length - 1); float t = s - i;
                Vector3 a = Boundary(i-1), b = Boundary(i), c = Boundary(i+1), d = Boundary(i+2);
                Vector3 center = .5f * (2*b + (-a+c)*t + (2*a-5*b+4*c-d)*t*t + (-a+3*b-3*c+d)*t*t*t);
                Quaternion q = Quaternion.Slerp(sections[i].transform.rotation, sections[Mathf.Min(i+1, sections.Length-1)].transform.rotation, t);
                for (int side = 0; side < 4; side++)
                {
                    float x = side == 0 || side == 3 ? -.5f : .5f;
                    float y = side < 2 ? -.5f : .5f;
                    Vector3 offset = new Vector3(x * RobotHydroDefinition.TailThickness, y * RobotHydroDefinition.TailHeight, 0f);
                    skinPoints[row*4+side] = skinFrame.InverseTransformPoint(center + q * offset);
                }
            }
            skin.vertices = skinPoints; skin.RecalculateNormals(); skin.RecalculateBounds();
        }
        public static Mesh BoxMesh()
        {
            // Reuse Unity's closed, outward-oriented primitive geometry without persisting an object.
            GameObject box = GameObject.CreatePrimitive(PrimitiveType.Cube);
            Mesh result = Instantiate(box.GetComponent<MeshFilter>().sharedMesh);
            box.SetActive(false);
            if(Application.isPlaying) Destroy(box); else DestroyImmediate(box);
            return result;
        }
        void OnDestroy() { foreach (Mesh mesh in transientMeshes) if (mesh != null) { if(Application.isPlaying) Destroy(mesh); else DestroyImmediate(mesh); } }
        void OnGUI()
        {
            if (!showHud) return;
            GUILayout.BeginArea(new Rect(18, 18, 390, 235), GUI.skin.box);
            GUILayout.Label("IRYS — simulation hydro / palme PVC rigide");
            GUILayout.Label("↑↓ : propulsion   ←→ : virage   Espace : arrêt");
            GUILayout.Label($"u = {u:F2}    v = {v:F2}");
            GUILayout.Label($"f = {Setpoint.frequencyHz:F2} Hz    A = {Setpoint.amplitudeDeg:F1}°    α moyen = {Setpoint.meanDeg:F1}°");
            GUILayout.Label($"Servo : {ServoAngle:F1}° / cible {ServoTarget:F1}°");
            GUILayout.Label($"Couple disponible ≤ {TorqueLimit:F2} N·m    vitesse robot {hull.linearVelocity.magnitude:F2} m/s");
            GUILayout.Label("350 × 70 × 1 mm — E effectif ≈ 3,34 GPa");
            GUILayout.Label("u = intensité, puissance en watts non calibrée");
            GUILayout.EndArea();
        }
    }
}
