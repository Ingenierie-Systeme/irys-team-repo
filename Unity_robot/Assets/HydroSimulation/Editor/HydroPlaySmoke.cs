using System;
using System.IO;
using UnityEngine;
using UnityEditor;
using UnityEditor.SceneManagement;
using Irys.Hydro;

[InitializeOnLoad]
public static class HydroPlaySmoke
{
    const string Key="IRYS_HYDRO_PLAY_SMOKE";
    static HydroRobot robot;
    static double started;
    static HydroPlaySmoke()
    {
        if(SessionState.GetBool(Key,false)) Register();
    }
    public static void Run()
    {
        EditorSceneManager.OpenScene(HydroSceneTools.ScenePath);
        SessionState.SetBool(Key,true);SessionState.SetString(Key+"error","");
        Register();EditorApplication.EnterPlaymode();
    }
    static void Register()
    {
        EditorApplication.playModeStateChanged-=State;EditorApplication.playModeStateChanged+=State;
        Application.logMessageReceived-=Log;Application.logMessageReceived+=Log;
    }
    static void Log(string message,string trace,LogType type)
    {
        if(type!=LogType.Error && type!=LogType.Exception && type!=LogType.Assert)return;
        // Unity's asynchronous editor search index is outside the simulation. Keep its diagnostic separately.
        if(trace.Contains("UnityEditor.Search."))
        {SessionState.SetString(Key+"editor",message+"\n"+trace);return;}
        string prior=SessionState.GetString(Key+"error","");
        if(prior.Length<16000)SessionState.SetString(Key+"error",prior+message+"\n"+trace+"\n");
    }
    static void State(PlayModeStateChange state)
    {
        if(state==PlayModeStateChange.EnteredPlayMode)
        {
            robot=UnityEngine.Object.FindAnyObjectByType<HydroRobot>();robot.keyboard=false;robot.u=.8f;robot.v=.4f;
            started=EditorApplication.timeSinceStartup;EditorApplication.update+=Tick;
        }
        if(state==PlayModeStateChange.EnteredEditMode)
        {
            SessionState.SetBool(Key,false);string error=SessionState.GetString(Key+"error","");
            Directory.CreateDirectory("../Documentation/HydroValidation");
            File.WriteAllText("../Documentation/HydroValidation/play-mode.txt",error.Length==0?"PASS — native Play mode, Start/FixedUpdate/LateUpdate, 8 seconds, no simulation errors.\n":error);
            if(error.Length==0)Debug.Log("HYDRO_PLAY_PASS");
            if(Application.isBatchMode)EditorApplication.Exit(error.Length==0?0:1);
        }
    }
    static void Tick()
    {
        if(robot!=null && robot.TimeSimulated>=8f)
        {
            if(!float.IsFinite(robot.hull.transform.position.x) || robot.Steps<2000)SessionState.SetString(Key+"error","Invalid native physics progress.");
            Stop();
        }
        else if(EditorApplication.timeSinceStartup-started>90f)
        {SessionState.SetString(Key+"error","Play mode timed out.");Stop();}
    }
    static void Stop() {EditorApplication.update-=Tick;EditorApplication.ExitPlaymode();}
}
