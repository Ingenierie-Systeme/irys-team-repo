using System;
using System.IO;
using UnityEngine;
using UnityEditor;
using UnityEditor.SceneManagement;
using Irys.Hydro;

public static class HydroPreview
{
    // Deterministic off-screen preview, using the same controller and physics as the playable scene.
    public static void Render()
    {
        const int width=1280,height=720,fps=25,frames=375;
        string output=Environment.GetEnvironmentVariable("IRYS_HYDRO_FRAMES") ?? "/tmp/irys-hydro-frames";
        Directory.CreateDirectory(output);Directory.CreateDirectory("../Documentation/HydroValidation");
        EditorSceneManager.OpenScene(HydroSceneTools.ScenePath);
        var robot=UnityEngine.Object.FindAnyObjectByType<HydroRobot>();robot.keyboard=false;robot.showHud=false;robot.Initialize();
        var camera=Camera.main;
        SimulationMode previous=Physics.simulationMode;Physics.simulationMode=SimulationMode.Script;Physics.SyncTransforms();
        var target=new RenderTexture(width,height,24,RenderTextureFormat.ARGB32);target.Create();
        var pixels=new Texture2D(width,height,TextureFormat.RGB24,false);
        camera.targetTexture=target;
        try
        {
            for(int frame=0;frame<frames;frame++)
            {
                float t=(float)frame/fps;
                robot.u=t<2f || t>=11f?0f:.8f;robot.v=t>=8f && t<11f?.7f:0f;
                for(int step=0;step<Mathf.RoundToInt(1f/fps/RobotHydroDefinition.StepSeconds);step++)
                {robot.Step(RobotHydroDefinition.StepSeconds);Physics.Simulate(RobotHydroDefinition.StepSeconds);}
                robot.RefreshVisual();camera.GetComponent<HydroCamera>().RefreshView();
                camera.Render();RenderTexture.active=target;
                pixels.ReadPixels(new Rect(0,0,width,height),0,0);pixels.Apply();RenderTexture.active=null;
                byte[] png=pixels.EncodeToPNG();File.WriteAllBytes(Path.Combine(output,$"frame-{frame:D4}.png"),png);
                if(frame==190)File.WriteAllBytes("../Documentation/HydroValidation/preview.png",png);
                if(frame%50==0)Debug.Log("HYDRO_PREVIEW_FRAME "+frame+"/"+frames);
            }
            Debug.Log("HYDRO_PREVIEW_OK "+output);
        }
        finally
        {
            camera.targetTexture=null;RenderTexture.active=null;target.Release();UnityEngine.Object.DestroyImmediate(target);UnityEngine.Object.DestroyImmediate(pixels);Physics.simulationMode=previous;
        }
    }
}
