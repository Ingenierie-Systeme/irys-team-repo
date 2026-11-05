using System;
using System.IO;
using System.Linq;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using Irys.Hydro;

public static class HydroPoolValidation
{
    public static void VerifyAndPlay() { Verify(); HydroPlaySmoke.Run(); }
    public static void BuildAndPlay() { HydroSceneTools.Build(); VerifyAndPlay(); }
    public static void Verify()
    {
        EditorSceneManager.OpenScene(HydroSceneTools.ScenePath);
        GameObject pool=GameObject.Find("Piscine_ENSTA_ROB");
        Require(pool!=null,"Pool prefab is present");
        Require(PrefabUtility.GetCorrespondingObjectFromSource(pool)!=null,"Pool remains a prefab instance");
        var filters=pool.GetComponentsInChildren<MeshFilter>(true);
        Require(filters.Length==24 && filters.All(f=>f.sharedMesh!=null),"All 24 original pool meshes are present");
        var renderers=pool.GetComponentsInChildren<Renderer>(true);
        Require(renderers.All(r=>r.sharedMaterials.All(m=>m!=null && m.shader!=null && m.shader.name!="Hidden/InternalErrorShader")),"Pool materials have valid shaders");
        string[] dependencies=AssetDatabase.GetDependencies(HydroPoolTools.PrefabPath,true);
        Require(!dependencies.Any(p=>p.EndsWith(".blend") || p.StartsWith("Assets/Modèles/")),"Pool does not depend on the local Blender source or legacy texture folder");
        int textures=dependencies.Count(p=>p.StartsWith("Assets/Environment/Pool/Textures/") && (p.EndsWith(".jpg") || p.EndsWith(".png")));
        Require(textures==6,"Six original albedo textures are shipped");
        var basin=filters.First(f=>f.name=="Cube");
        HydroPoolTools.Measure(basin,RobotHydroDefinition.WaterLevel,out Bounds floor,out Bounds surface);
        Require(surface.size.x>10f && surface.size.z>15f && floor.center.y< -6f,"Measured basin dimensions match the original asset");
        var water=GameObject.Find("Surface_eau_Y0");
        Require(water!=null && Mathf.Abs(water.transform.position.y-RobotHydroDefinition.WaterLevel)<1e-6f,"Visual and physical water levels coincide");
        Require((water.GetComponent<Renderer>().bounds.size-new Vector3(surface.size.x,0,surface.size.z)).magnitude<.001f,"Water surface fits the basin");
        Require(GameObject.Find("Fond_du_bassin")==null && GameObject.Find("Reperes_fond_50cm")==null,"Simplified pool has been removed");
        var limits=GameObject.Find("Limites_piscine");
        Require(limits!=null && limits.GetComponentsInChildren<BoxCollider>().Length==5,"Four walls and one bottom collider are present");
        Physics.SyncTransforms();
        Vector3 origin=new Vector3(surface.center.x,-1f,surface.center.z);
        foreach(Vector3 direction in new[]{Vector3.left,Vector3.right,Vector3.back,Vector3.forward,Vector3.down})
        {
            Require(Physics.Raycast(origin,direction,out RaycastHit hit,30f) && hit.collider.transform.IsChildOf(limits.transform),"Pool collision boundary along "+direction);
        }
        var camera=Camera.main.GetComponent<HydroCamera>();
        Require(camera.target!=null && camera.overview,"Camera opens on the pool overview");
        camera.RefreshView();Require(Vector3.Distance(camera.transform.position,camera.overviewPosition)<.001f,"Pool overview camera position");
        camera.overview=false;camera.RefreshView();
        Require(Vector3.Distance(camera.transform.position,camera.target.position)<3f,"Robot follow view is available");
        Directory.CreateDirectory("../Documentation/HydroValidation");
        File.WriteAllText("../Documentation/HydroValidation/pool-validation.json",JsonUtility.ToJson(new Report {
            status="PASS",prefab=HydroPoolTools.PrefabPath,meshes=filters.Length,vertices=filters.Sum(f=>f.sharedMesh.vertexCount),textures=textures,
            waterLevel=surface.center.y,width=surface.size.x,length=surface.size.z,depth=surface.center.y-floor.center.y,
            portableWithoutBlender=true,collisionBoundaries=5
        },true));
        Debug.Log("HYDRO_POOL_VALIDATION_PASS");
    }
    static void Require(bool condition,string message) { if(!condition)throw new Exception("HYDRO_POOL_FAIL: "+message); }
    [Serializable] sealed class Report { public string status,prefab;public int meshes,vertices,textures,collisionBoundaries;public float waterLevel,width,length,depth;public bool portableWithoutBlender; }
}
