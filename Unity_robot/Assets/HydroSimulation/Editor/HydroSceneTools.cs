using System;
using System.IO;
using System.Linq;
using System.Collections.Generic;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;
using Irys.Hydro;

public static class HydroSceneTools
{
    public const string ScenePath = "Assets/Scenes/SimulationHydro.unity";
    const string Generated = "Assets/HydroSimulation/Generated/";
    sealed class Visual { public Mesh mesh; public Matrix4x4 matrix; public string name; public bool fin; }

    [MenuItem("IRYS/Hydro/Construire la scène de référence")]
    public static void Build()
    {
        if (File.Exists(ScenePath)) throw new InvalidOperationException("Scene already exists; preserve edits before rebuilding.");
        EditorSceneManager.OpenScene("Assets/Scenes/SampleScene.unity");
        Transform original = GameObject.Find("Robot").transform;
        var tube = original.GetComponentsInChildren<MeshFilter>(true).First(f => f.name == "tube" && f.sharedMesh != null);
        Vector3 center = original.InverseTransformPoint(tube.GetComponent<Renderer>().bounds.center);
        var visuals = new List<Visual>();
        // Preserve the source CAO convention: +Z forward, tail toward -Z.
        Matrix4x4 canonical = Matrix4x4.Translate(-center) * original.worldToLocalMatrix;
        foreach (var filter in original.GetComponentsInChildren<MeshFilter>(true))
        {
            if (filter.sharedMesh == null) continue;
            bool tail = false, fin = false;
            for (Transform t = filter.transform; t != original; t = t.parent)
            {
                tail |= t.name.StartsWith("nageoire", StringComparison.OrdinalIgnoreCase) || t.name.StartsWith("articulation", StringComparison.OrdinalIgnoreCase);
                fin |= t.name.StartsWith("aileron", StringComparison.OrdinalIgnoreCase);
            }
            if (!tail) visuals.Add(new Visual { mesh = filter.sharedMesh, matrix = canonical * filter.transform.localToWorldMatrix, name = filter.name, fin = fin });
        }
        if (!visuals.Any(x => x.fin)) throw new InvalidOperationException("No CAO fins in source robot.");
        Scene scene = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
        Directory.CreateDirectory(Generated); AssetDatabase.Refresh();
        Material metal = MaterialAsset("Metal", new Color(.28f,.34f,.39f), .65f);
        Material pvc = MaterialAsset("PVC", new Color(.88f,.68f,.12f), .05f);
        Material finMaterial = MaterialAsset("Ailerons", new Color(.06f,.42f,.42f), .1f);
        Material tubeMaterial = MaterialAsset("Tube", new Color(.45f,.72f,.78f), .15f);
        GameObject root = new GameObject("IRYS_Hydro"); root.SetActive(false);
        root.transform.position = new Vector3(0,-.034f,0);
        var hull = Body(root, RobotHydroDefinition.HullMass, new Vector3(.11f,.11f,.4f));
        hull.centerOfMass = new Vector3(0,-.012f,.008f);
        // Cylinder transverse inertia, closed tube envelope. Aggregate equipment mass is included.
        float r = RobotHydroDefinition.HullRadius, l = RobotHydroDefinition.HullLength, m = hull.mass;
        hull.inertiaTensor = new Vector3(m*(3*r*r+l*l)/12f,m*(3*r*r+l*l)/12f,.5f*m*r*r);
        var collider = root.AddComponent<CapsuleCollider>(); collider.direction=2; collider.radius=r; collider.height=l;
        var controller = root.AddComponent<HydroRobot>(); controller.hull = hull; controller.tailMaterial = pvc;
        var fins = new List<MeshFilter>();
        for (int i=0;i<visuals.Count;i++)
        {
            Visual v = visuals[i];
            // Reference the original meshes, including the fins, rather than duplicating the CAO.
            if(v.matrix.determinant<0f) throw new InvalidOperationException("Mirrored source transform requires an explicit geometry conversion.");
            GameObject visual = new GameObject(v.fin ? "Aileron_CAO_" + i : v.name);
            visual.transform.SetParent(root.transform,false);
            visual.transform.localPosition=v.matrix.GetColumn(3);visual.transform.localRotation=v.matrix.rotation;visual.transform.localScale=v.matrix.lossyScale;
            var filter=visual.AddComponent<MeshFilter>(); filter.sharedMesh=v.mesh;
            var material=v.fin ? finMaterial : v.name=="tube" ? tubeMaterial : metal;
            visual.AddComponent<MeshRenderer>().sharedMaterials=Enumerable.Repeat(material,v.mesh.subMeshCount).ToArray();
            if(v.fin) fins.Add(filter);
        }
        controller.fixedFins=fins.ToArray();
        // Coarse cylinder surface is a separate, invisible hydro proxy, never an extra mass/volume.
        GameObject proxy=GameObject.CreatePrimitive(PrimitiveType.Cylinder); proxy.name="Enveloppe_hydro_tube";
        proxy.transform.SetParent(root.transform,false); proxy.transform.localRotation=Quaternion.Euler(90,0,0);
        proxy.transform.localScale=new Vector3(2*r,l*.5f,2*r);
        UnityEngine.Object.DestroyImmediate(proxy.GetComponent<Collider>()); proxy.GetComponent<Renderer>().enabled=false;
        controller.hullSurface=proxy.GetComponent<MeshFilter>();
        GameObject servoObject=new GameObject("Servo_D955TW"); servoObject.transform.SetParent(root.transform,false);
        servoObject.transform.localPosition=new Vector3(0,0,-.325f);
        var servo=Body(servoObject,RobotHydroDefinition.ServoMass,Vector3.one*.04f);
        Revolute(servo,hull,RobotHydroDefinition.TravelDeg,8f,.15f,RobotHydroDefinition.StallTorque);
        controller.servo=servo;
        var arm=GameObject.CreatePrimitive(PrimitiveType.Cube);arm.name="Bras_transmission";arm.transform.SetParent(servoObject.transform,false);
        arm.transform.localPosition=new Vector3(0,.014f,-.04f);arm.transform.localScale=new Vector3(.02f,.015f,.08f);
        UnityEngine.Object.DestroyImmediate(arm.GetComponent<Collider>());arm.GetComponent<Renderer>().sharedMaterial=metal;
        controller.sections=new ArticulationBody[RobotHydroDefinition.Sections];
        float ds=RobotHydroDefinition.TailLength/RobotHydroDefinition.Sections;
        float segmentMass=RobotHydroDefinition.PvcDensity*RobotHydroDefinition.TailLength*RobotHydroDefinition.TailHeight*RobotHydroDefinition.TailThickness/RobotHydroDefinition.Sections;
        ArticulationBody previous=servo;
        for(int i=0;i<controller.sections.Length;i++)
        {
            GameObject section=new GameObject("PVC_section_"+i); section.transform.SetParent(previous.transform,false);
            section.transform.localPosition=i==0?new Vector3(0,.014f,-.08f):Vector3.back*ds;
            var body=Body(section,segmentMass,new Vector3(RobotHydroDefinition.TailThickness,RobotHydroDefinition.TailHeight,ds));
            body.centerOfMass=Vector3.back*ds*.5f;
            float stiffness=RobotHydroDefinition.BendingRigidity/ds*(i==0?2f:1f);
            // Damping based on downstream bending inertia, with a moderate modal damping target.
            float inertia=0f; for(int j=0;j<controller.sections.Length-i;j++) inertia+=segmentMass*(Mathf.Pow((j+.5f)*ds,2)+ds*ds/12f);
            float damping=2f*.3f*Mathf.Sqrt(stiffness*inertia);
            Revolute(body,previous,55f,stiffness,damping,100f);
            controller.sections[i]=body; previous=body;
        }
        Time.fixedDeltaTime=RobotHydroDefinition.StepSeconds;
        root.SetActive(true);
        var light=new GameObject("Soleil").AddComponent<Light>(); light.type=LightType.Directional; light.intensity=1.5f; light.transform.rotation=Quaternion.Euler(45,-25,0);
        RenderSettings.ambientMode=UnityEngine.Rendering.AmbientMode.Flat;
        RenderSettings.ambientLight=new Color(.65f,.72f,.78f);
        GameObject cameraObject=new GameObject("Camera_Hydro"); var camera=cameraObject.AddComponent<Camera>(); camera.tag="MainCamera";
        camera.nearClipPlane=.02f; camera.farClipPlane=100; camera.fieldOfView=43f; camera.backgroundColor=new Color(.06f,.18f,.25f); camera.clearFlags=CameraClearFlags.SolidColor;
        cameraObject.AddComponent<HydroCamera>().target=root.transform;
        cameraObject.transform.position=root.transform.position+new Vector3(1.05f,.8f,-1.5f); cameraObject.transform.LookAt(root.transform.position+Vector3.back*.15f);
        Material waterMaterial=MaterialAsset("Eau",new Color(.10f,.55f,.66f,.12f),.3f);
        waterMaterial.SetFloat("_Surface",1);waterMaterial.SetFloat("_SrcBlend",(float)UnityEngine.Rendering.BlendMode.SrcAlpha);waterMaterial.SetFloat("_DstBlend",(float)UnityEngine.Rendering.BlendMode.OneMinusSrcAlpha);
        waterMaterial.SetFloat("_ZWrite",0);waterMaterial.EnableKeyword("_SURFACE_TYPE_TRANSPARENT");waterMaterial.renderQueue=3000;
        HydroPoolTools.Attach(scene);
        EditorSceneManager.SaveScene(scene,ScenePath);
        EditorBuildSettings.scenes=EditorBuildSettings.scenes.Where(s=>s.path!=ScenePath).Concat(new[]{new EditorBuildSettingsScene(ScenePath,true)}).ToArray();
        AssetDatabase.SaveAssets();
        Debug.Log("HYDRO_BUILD_OK fins="+fins.Count+" sourceTubeCenter="+center.ToString("F4")+" E="+RobotHydroDefinition.YoungModulus);
    }


    static Material MaterialAsset(string name,Color color,float metallic)
    {
        var material=AssetDatabase.LoadAssetAtPath<Material>(Generated+name+".mat");
        bool create=material==null;
        if(create)material=new Material(Shader.Find("Universal Render Pipeline/Lit"));material.name=name;
        material.SetColor("_BaseColor",color);material.SetFloat("_Metallic",metallic);material.SetFloat("_Smoothness",.45f);
        if(create)AssetDatabase.CreateAsset(material,Generated+name+".mat");else EditorUtility.SetDirty(material);
        return material;
    }
    static ArticulationBody Body(GameObject obj,float mass,Vector3 size)
    {
        var body=obj.AddComponent<ArticulationBody>();body.mass=mass;body.useGravity=true;
        body.automaticCenterOfMass=false;body.automaticInertiaTensor=false;body.centerOfMass=Vector3.zero;
        body.inertiaTensorRotation=Quaternion.identity;
        body.inertiaTensor=mass/12f*new Vector3(size.y*size.y+size.z*size.z,size.x*size.x+size.z*size.z,size.x*size.x+size.y*size.y);
        body.linearDamping=0f;body.angularDamping=0f;body.jointFriction=0f;
        body.solverIterations=24;body.solverVelocityIterations=12;
        return body;
    }
    static void Revolute(ArticulationBody body,ArticulationBody parent,float limit,float stiffness,float damping,float torque)
    {
        body.jointType=ArticulationJointType.RevoluteJoint;
        body.matchAnchors=false;body.anchorPosition=Vector3.zero;body.anchorRotation=Quaternion.FromToRotation(Vector3.right,Vector3.up);
        body.parentAnchorPosition=parent.transform.InverseTransformPoint(body.transform.position);body.parentAnchorRotation=body.anchorRotation;
        body.twistLock=ArticulationDofLock.LimitedMotion;body.swingYLock=ArticulationDofLock.LockedMotion;body.swingZLock=ArticulationDofLock.LockedMotion;
        var drive=body.xDrive;drive.lowerLimit=-limit;drive.upperLimit=limit;drive.stiffness=stiffness;drive.damping=damping;drive.forceLimit=torque;drive.target=0;drive.targetVelocity=0;
        drive.driveType=ArticulationDriveType.Force;body.xDrive=drive;
    }
    public static void Inspect()
    {
        EditorSceneManager.OpenScene("Assets/Scenes/SampleScene.unity");
        var robot = GameObject.Find("Robot");
        var lines = new List<string>();
        foreach (var r in robot.GetComponentsInChildren<MeshFilter>(true))
        {
            var renderer = r.GetComponent<Renderer>();
            lines.Add(r.name + " parent=" + r.transform.parent.name + " p=" + robot.transform.InverseTransformPoint(r.transform.position).ToString("F4")
                + " scale=" + r.transform.lossyScale.ToString("F4") + " bounds=" + (renderer == null ? "none" : renderer.bounds.ToString("F4")) + " mesh=" + (r.sharedMesh == null ? "MISSING" : r.sharedMesh.name));
        }
        Directory.CreateDirectory("../Documentation");
        File.WriteAllLines("../Documentation/hydro-source-inspection.txt", lines);
        Debug.Log("HYDRO_INSPECT_OK");
    }
}
