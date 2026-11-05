using System;
using System.Collections.Generic;
using System.IO;
using System.IO.Compression;
using System.Linq;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;
using Object = UnityEngine.Object;

// Exact mesh/UV export of the supplied pool. The shipped prefab has no Blender dependency.
public static class HydroPoolTools
{
    const string Folder = "Assets/Environment/Pool/";
    public const string PrefabPath = Folder + "Piscine_ENSTA_ROB.prefab";
    const string Source = "Assets/Modèles/Piscine_ENSTA_ROB_with_textures.blend";

    public static void ExportAndIntegrate()
    {
        ExportOriginal();
        Integrate();
    }

    public static void ExportOriginal()
    {
        if (File.Exists(PrefabPath)) throw new InvalidOperationException("Pool prefab already exported; preserve it before exporting again.");
        var model = AssetDatabase.LoadAssetAtPath<GameObject>(Source);
        if (model == null) throw new InvalidOperationException("Import the supplied pool Blender asset before exporting.");
        Directory.CreateDirectory(Folder + "Meshes");
        Directory.CreateDirectory(Folder + "Materials");
        Directory.CreateDirectory(Folder + "Textures");
        AssetDatabase.Refresh();
        var instance = (GameObject)PrefabUtility.InstantiatePrefab(model);
        instance.name = "Piscine_ENSTA_ROB";
        PrefabUtility.UnpackPrefabInstance(instance, PrefabUnpackMode.Completely, InteractionMode.AutomatedAction);
        var meshMap = new Dictionary<Mesh, Mesh>();
        var materialMap = new Dictionary<Material, Material>();
        var textureMap = new Dictionary<Texture, Texture>();
        foreach (MeshFilter filter in instance.GetComponentsInChildren<MeshFilter>(true))
        {
            if (filter.sharedMesh == null) continue;
            Mesh source = filter.sharedMesh;
            if (!meshMap.TryGetValue(source, out Mesh mesh))
            {
                mesh = Object.Instantiate(source);
                mesh.name = source.name;
                mesh.hideFlags = HideFlags.None;
                string path = Folder + "Meshes/Pool_" + meshMap.Count.ToString("D2") + ".poolmesh";
                string temporary = Path.Combine(Path.GetTempPath(), "irys-pool-export-" + Guid.NewGuid().ToString("N") + ".asset");
                try
                {
                    UnityEditorInternal.InternalEditorUtility.SaveToSerializedFileAndForget(new Object[] { mesh }, temporary, false);
                    using(var sourceFile=File.OpenRead(temporary))
                    using(var targetFile=File.Create(path))
                    using(var gzip=new GZipStream(targetFile,System.IO.Compression.CompressionLevel.Optimal))sourceFile.CopyTo(gzip);
                }
                finally {if(File.Exists(temporary))File.Delete(temporary);}
                AssetDatabase.ImportAsset(path, ImportAssetOptions.ForceSynchronousImport);
                Object.DestroyImmediate(mesh);
                mesh = AssetDatabase.LoadAssetAtPath<Mesh>(path);
                if(mesh==null || mesh.vertexCount!=source.vertexCount)throw new InvalidOperationException("Native pool mesh export failed: "+source.name);
                meshMap.Add(source, mesh);
            }
            filter.sharedMesh = mesh;
        }
        AssetDatabase.SaveAssets();
        foreach (Renderer renderer in instance.GetComponentsInChildren<Renderer>(true))
        {
            Material[] materials = renderer.sharedMaterials;
            for (int i = 0; i < materials.Length; i++)
            {
                Material source = materials[i];
                if (source == null) continue;
                source = AssetDatabase.LoadAssetAtPath<Material>("Assets/Modèles/PoolMaterials/" + source.name + ".mat") ?? source;
                if (!materialMap.TryGetValue(source, out Material material))
                {
                    material = new Material(source) { name = source.name };
                    Texture baseMap = source.HasProperty("_BaseMap") ? source.GetTexture("_BaseMap") : null;
                    // Keep the supplied albedo and UVs. Clear absent heavy EXR maps so every dependency ships.
                    foreach (string property in material.GetTexturePropertyNames()) material.SetTexture(property, null);
                    material.DisableKeyword("_NORMALMAP"); material.DisableKeyword("_PARALLAXMAP");
                    material.DisableKeyword("_METALLICSPECGLOSSMAP"); material.DisableKeyword("_OCCLUSIONMAP");
                    if (baseMap != null)
                    {
                        if (!textureMap.TryGetValue(baseMap, out Texture texture))
                        {
                            string path = AssetDatabase.GetAssetPath(baseMap);
                            string exported = Folder + "Textures/" + Path.GetFileName(path);
                            File.Copy(path, exported, true);
                            AssetDatabase.ImportAsset(exported, ImportAssetOptions.ForceSynchronousImport);
                            var importer = (TextureImporter)AssetImporter.GetAtPath(exported);
                            importer.maxTextureSize = 2048; importer.wrapMode = TextureWrapMode.Repeat;
                            importer.SaveAndReimport();
                            texture = AssetDatabase.LoadAssetAtPath<Texture>(exported);
                            textureMap.Add(baseMap, texture);
                        }
                        material.SetTexture("_BaseMap", texture);
                        if (material.HasProperty("_MainTex")) material.SetTexture("_MainTex", texture);
                    }
                    AssetDatabase.CreateAsset(material, Folder + "Materials/" + source.name + ".mat");
                    materialMap.Add(source, material);
                }
                materials[i] = material;
            }
            renderer.sharedMaterials = materials;
        }
        foreach (var collider in instance.GetComponentsInChildren<Collider>(true)) Object.DestroyImmediate(collider);
        foreach (var camera in instance.GetComponentsInChildren<Camera>(true)) camera.enabled = false;
        foreach (var light in instance.GetComponentsInChildren<Light>(true)) light.enabled = false;
        foreach (var listener in instance.GetComponentsInChildren<AudioListener>(true)) listener.enabled = false;
        foreach (var animator in instance.GetComponentsInChildren<Animator>(true)) Object.DestroyImmediate(animator);
        foreach (var animation in instance.GetComponentsInChildren<Animation>(true)) Object.DestroyImmediate(animation);
        PrefabUtility.SaveAsPrefabAsset(instance, PrefabPath);
        Object.DestroyImmediate(instance);
        AssetDatabase.SaveAssets();
        Debug.Log($"HYDRO_POOL_EXPORTED meshes={meshMap.Count} materials={materialMap.Count} textures={textureMap.Count}");
    }

    [MenuItem("IRYS/Hydro/Intégrer la piscine CAO")]
    public static void Integrate()
    {
        Scene scene = EditorSceneManager.OpenScene(HydroSceneTools.ScenePath);
        Attach(scene);
        EditorSceneManager.MarkSceneDirty(scene);
        EditorSceneManager.SaveScene(scene);
        AssetDatabase.SaveAssets();
        Debug.Log("HYDRO_POOL_INTEGRATED");
    }

    public static void Attach(Scene scene)
    {
        var prefab = AssetDatabase.LoadAssetAtPath<GameObject>(PrefabPath);
        if (prefab == null) throw new InvalidOperationException("The native pool prefab must be present.");
        foreach (string name in new[] { "Fond_du_bassin", "Reperes_fond_50cm", "Piscine_ENSTA_ROB", "Limites_piscine" })
        {
            GameObject old = GameObject.Find(name);
            if (old != null) Object.DestroyImmediate(old);
        }
        var pool = (GameObject)PrefabUtility.InstantiatePrefab(prefab, scene);
        pool.name = "Piscine_ENSTA_ROB";
        MeshFilter basin = pool.GetComponentsInChildren<MeshFilter>(true).First(f => f.name == "Cube");
        Measure(basin, -.35f, out Bounds originalFloor, out Bounds originalSurface);
        pool.transform.position += new Vector3(-originalSurface.center.x, .35f, -originalSurface.center.z);
        Measure(basin, Irys.Hydro.RobotHydroDefinition.WaterLevel, out Bounds floor, out Bounds surface);
        GameObject water = GameObject.Find("Surface_eau_Y0");
        if (water == null) water = GameObject.CreatePrimitive(PrimitiveType.Plane);
        water.name = "Surface_eau_Y0";
        if (water.GetComponent<Collider>() != null) Object.DestroyImmediate(water.GetComponent<Collider>());
        water.transform.SetPositionAndRotation(surface.center, Quaternion.identity);
        water.transform.localScale = new Vector3(surface.size.x / 10f, 1f, surface.size.z / 10f);
        var renderer = water.GetComponent<Renderer>();
        renderer.sharedMaterial = AssetDatabase.LoadAssetAtPath<Material>("Assets/HydroSimulation/Generated/Eau.mat");
        renderer.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
        renderer.receiveShadows = false;
        var limits = new GameObject("Limites_piscine");
        const float thickness = .12f;
        Boundary(limits.transform, "Fond", new Vector3(surface.center.x, floor.center.y-thickness/2, surface.center.z), new Vector3(surface.size.x+2*thickness,thickness,surface.size.z+2*thickness));
        float height=surface.center.y-floor.center.y+.25f, mid=(surface.center.y+.25f+floor.center.y)/2f;
        Boundary(limits.transform,"Gauche",new Vector3(surface.min.x-thickness/2,mid,surface.center.z),new Vector3(thickness,height,surface.size.z));
        Boundary(limits.transform,"Droite",new Vector3(surface.max.x+thickness/2,mid,surface.center.z),new Vector3(thickness,height,surface.size.z));
        Boundary(limits.transform,"Arriere",new Vector3(surface.center.x,mid,surface.min.z-thickness/2),new Vector3(surface.size.x,height,thickness));
        Boundary(limits.transform,"Avant",new Vector3(surface.center.x,mid,surface.max.z+thickness/2),new Vector3(surface.size.x,height,thickness));
        var camera = Camera.main.GetComponent<Irys.Hydro.HydroCamera>();
        Camera sourceCamera = pool.GetComponentsInChildren<Camera>(true).FirstOrDefault();
        camera.overviewPosition = sourceCamera == null ? new Vector3(7.75f,5.85f,-8f) : new Vector3(sourceCamera.transform.position.x,5.85f,sourceCamera.transform.position.z);
        camera.overviewTarget = surface.center + Vector3.down*.45f;
        camera.overview = true;
        camera.RefreshView();
        Directory.CreateDirectory("../Documentation/HydroValidation");
        File.WriteAllText("../Documentation/HydroValidation/pool.json",JsonUtility.ToJson(new PoolReport {
            prefab=PrefabPath,waterLevel=surface.center.y,waterCenter=surface.center,waterSize=surface.size,floorY=floor.center.y,
            meshes=pool.GetComponentsInChildren<MeshFilter>(true).Length, overview=camera.overviewPosition
        },true));
    }

    static void Boundary(Transform parent,string name,Vector3 center,Vector3 size)
    { var obj=new GameObject(name);obj.transform.SetParent(parent,false);obj.transform.position=center;obj.AddComponent<BoxCollider>().size=size; }

    public static void Measure(MeshFilter filter,float level,out Bounds floor,out Bounds surface)
    {
        Mesh mesh=filter.sharedMesh;
        Vector3[] points=mesh.vertices.Select(filter.transform.TransformPoint).ToArray();
        int sub=Array.FindIndex(filter.GetComponent<Renderer>().sharedMaterials,m=>m!=null && m.name=="brown_floor_tiles");
        if(sub<0)throw new InvalidOperationException("Pool tile material not found.");
        int[] tile=mesh.GetTriangles(sub);
        float bottom=tile.Min(i=>points[i].y);
        Vector3[] bottomPoints=tile.Select(i=>points[i]).Where(p=>Mathf.Abs(p.y-bottom)<.002f).ToArray();
        floor=new Bounds(bottomPoints[0],Vector3.zero);foreach(var point in bottomPoints)floor.Encapsulate(point);
        Vector3 origin=new Vector3(floor.center.x,level,floor.center.z);
        int[] triangles=mesh.triangles;
        float left=RayDistance(origin,Vector3.left,points,triangles),right=RayDistance(origin,Vector3.right,points,triangles);
        float back=RayDistance(origin,Vector3.back,points,triangles),forward=RayDistance(origin,Vector3.forward,points,triangles);
        if(!float.IsFinite(left+right+back+forward))throw new InvalidOperationException("Pool wall intersections missing.");
        surface=new Bounds();surface.SetMinMax(origin+new Vector3(-left,0,-back),origin+new Vector3(right,0,forward));
        surface.Expand(new Vector3(-.04f,0,-.04f));
        if(surface.size.x<1f || surface.size.z<1f || bottom>=level)throw new InvalidOperationException("Invalid measured pool dimensions.");
    }
    static float RayDistance(Vector3 origin,Vector3 direction,Vector3[] points,int[] triangles)
    {
        float nearest=float.PositiveInfinity;
        for(int i=0;i<triangles.Length;i+=3)
        {
            Vector3 a=points[triangles[i]],e1=points[triangles[i+1]]-a,e2=points[triangles[i+2]]-a;
            Vector3 cross=Vector3.Cross(direction,e2);float determinant=Vector3.Dot(e1,cross);
            if(Mathf.Abs(determinant)<1e-8f)continue;
            float inverse=1f/determinant;Vector3 delta=origin-a;
            float u=Vector3.Dot(delta,cross)*inverse;Vector3 q=Vector3.Cross(delta,e1);
            float v=Vector3.Dot(direction,q)*inverse,distance=Vector3.Dot(e2,q)*inverse;
            if(u>=0 && v>=0 && u+v<=1 && distance>.001f)nearest=Mathf.Min(nearest,distance);
        }
        return nearest;
    }
    [Serializable] sealed class PoolReport { public string prefab;public float waterLevel,floorY;public Vector3 waterCenter,waterSize,overview;public int meshes; }
}
