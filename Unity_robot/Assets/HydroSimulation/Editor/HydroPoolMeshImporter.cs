using System;
using System.IO;
using System.IO.Compression;
using UnityEditor.AssetImporters;
using UnityEngine;

// Lossless source storage for the original pool meshes. Unity builds ordinary Mesh assets.
// Compression avoids committing huge hex-encoded YAML vertex buffers or requiring Blender at import.
[ScriptedImporter(1, "poolmesh")]
public sealed class HydroPoolMeshImporter : ScriptedImporter
{
    public override void OnImportAsset(AssetImportContext context)
    {
        string temporary = Path.Combine(Path.GetTempPath(), "irys-pool-" + Guid.NewGuid().ToString("N") + ".asset");
        try
        {
            using (var source = File.OpenRead(context.assetPath))
            using (var gzip = new GZipStream(source, CompressionMode.Decompress))
            using (var target = File.Create(temporary)) gzip.CopyTo(target);
            var objects = UnityEditorInternal.InternalEditorUtility.LoadSerializedFileAndForget(temporary);
            Mesh mesh = null;
            foreach (var obj in objects)
                if (obj is Mesh found && mesh == null) mesh = found;
                else UnityEngine.Object.DestroyImmediate(obj);
            if (mesh == null) throw new InvalidDataException("Pool source contains no mesh.");
            mesh.hideFlags = HideFlags.None;
            context.AddObjectToAsset("mesh", mesh);
            context.SetMainObject(mesh);
        }
        finally { if (File.Exists(temporary)) File.Delete(temporary); }
    }
}
