using System;
using System.IO;
using System.Text;
using System.Globalization;
using UnityEngine;
using UnityEditor;
using UnityEditor.SceneManagement;
using Irys.Hydro;

public static class HydroValidation
{
    [Serializable] public sealed class Trial
    {
        public string name;
        public float dt, seconds, forward, lateral, yaw, finalSpeed, maxSpeed, maxJointDeg, maxServoDeg, maxTrackingDeg, maxTargetSpeed, maxTargetAcceleration, lastRestSpeed, peakAngularSpeed;
        public Vector3 finalPosition;
        public double wallSeconds;
    }
    [Serializable] sealed class Report { public string status; public float youngModulus; public int assertions; public Trial[] trials; }
    static int checks;
    static void Verify(bool ok,string message) { if(!ok) throw new Exception("HYDRO_FAIL: "+message); checks++; }
    public static void RunAndPlay() { Run(); HydroPlaySmoke.Run(); }

    [MenuItem("IRYS/Hydro/Valider commande et dynamique")]
    public static void Run()
    {
        Directory.CreateDirectory("../Documentation/HydroValidation");
        SimulationMode previous=Physics.simulationMode;
        try
        {
            checks=0;Physics.simulationMode=SimulationMode.Script;
            CheckCommand();CheckBuoyancy();
            float dt=RobotHydroDefinition.StepSeconds;
            var rest=Simulate("rest",dt,12f,0f,0f,false);
            var straight=Simulate("straight_stop",dt,22f,.8f,0f,true);
            var left=Simulate("turn_positive",dt,16f,.8f,.7f,false);
            var right=Simulate("turn_negative",dt,16f,.8f,-.7f,false);
            var full=Simulate("full_command",dt,16f,1f,1f,false);
            var refined=Simulate("straight_stop_half_step",dt*.5f,22f,.8f,0f,true);
            Verify(rest.finalSpeed<.04f,"rest settles below 4 cm/s");
            Verify(straight.forward>.03f,"positive forward propulsion");
            Verify(left.yaw*right.yaw<0f && Mathf.Abs(left.yaw)>3f && Mathf.Abs(right.yaw)>3f,"opposite yaw response");
            Verify(Mathf.Abs(straight.forward-refined.forward)<Mathf.Max(.05f,Mathf.Abs(refined.forward)*.15f),"forward displacement converges to 15% or 5 cm");
            var report=new Report { status="PASS",youngModulus=RobotHydroDefinition.YoungModulus,assertions=checks,trials=new[]{rest,straight,left,right,full,refined} };
            if(File.Exists("../Documentation/HydroValidation/failure.txt"))File.Delete("../Documentation/HydroValidation/failure.txt");
            File.WriteAllText("../Documentation/HydroValidation/report.json",JsonUtility.ToJson(report,true));
            EditorSceneManager.OpenScene(HydroSceneTools.ScenePath);
            Debug.Log("HYDRO_VALIDATION_PASS assertions="+checks);
        }
        catch(Exception e)
        {
            File.WriteAllText("../Documentation/HydroValidation/failure.txt",e.ToString());
            Debug.LogException(e);if(Application.isBatchMode) EditorApplication.Exit(1);else throw;
        }
        finally { Physics.simulationMode=previous; }
    }

    static void CheckCommand()
    {
        float priorA=-1f,priorF=-1f;
        for(int i=0;i<=100;i++) for(int j=-10;j<=10;j++)
        {
            float u=i/100f, mean=j/10f*RobotHydroDefinition.MaxMeanDeg;
            var s=HydroCommand.Map(u,mean);
            Verify(s.amplitudeDeg+Mathf.Abs(s.meanDeg)<=RobotHydroDefinition.TravelDeg+.0001f,"command travel");
            Verify(2*Mathf.PI*s.frequencyHz*s.amplitudeDeg+2*RobotHydroDefinition.MaxMeanDeg/RobotHydroDefinition.MeanFilterSeconds<=RobotHydroDefinition.CommandSpeedDeg+.001f,"command velocity envelope");
            Verify(Mathf.Abs(Mathf.Pow(s.amplitudeDeg/RobotHydroDefinition.MaxAmplitudeDeg,2)+Mathf.Pow(1-s.frequencyHz/RobotHydroDefinition.MaxFrequencyHz,2)-1)<1e-4f,"elliptical command arc");
            if(j==0) { Verify(s.amplitudeDeg>=priorA && s.frequencyHz>=priorF,"monotone arc"); priorA=s.amplitudeDeg;priorF=s.frequencyHz; }
        }
        var a=new HydroCommand();var b=new HydroCommand();HydroSetpoint sa=default,sb=default;
        for(int i=0;i<100;i++)sa=a.Step(1,.7f,.01f);
        for(int i=0;i<200;i++)sb=b.Step(1,.7f,.005f);
        Verify(Mathf.Abs(sa.meanDeg-sb.meanDeg)<.001f && Mathf.Abs(sa.frequencyHz-sb.frequencyHz)<.0001f,"filter timestep invariance");
        Verify(Mathf.Abs(RobotHydroDefinition.YoungModulus/1e9f-3.33736f)<.001f,"cantilever identification");
        var random=new System.Random(37);
        for(int i=0;i<1000;i++)
        {
            var v=new Vector3((float)random.NextDouble()*4-2,(float)random.NextDouble()*4-2,(float)random.NextDouble()*4-2);
            var n=new Vector3((float)random.NextDouble()-.5f,(float)random.NextDouble()-.5f,(float)random.NextDouble()-.5f).normalized;
            Verify(Vector3.Dot(HydroSurface.Drag(v,n,.2f,1.8f,.008f),v)<=1e-5f,"passive fluid dissipation");
        }
    }
    static void CheckBuoyancy()
    {
        var go=GameObject.CreatePrimitive(PrimitiveType.Cube);go.transform.localScale=Vector3.one*.1f;
        var body=go.AddComponent<ArticulationBody>();
        var s=new HydroSurface(go.GetComponent<MeshFilter>().sharedMesh,go.transform,body,buoyant:true);
        go.transform.position=Vector3.down*.1f;Physics.SyncTransforms();s.Step(Vector3.zero);
        Verify(Mathf.Abs(s.DisplacedVolume-.001f)<1e-7f,"closed submerged volume");
        go.transform.position=Vector3.zero;Physics.SyncTransforms();s.Step(Vector3.zero);
        Verify(Mathf.Abs(s.DisplacedVolume-.0005f)<1e-7f,"half immersed clipped volume");
        go.transform.position=Vector3.up*.1f;Physics.SyncTransforms();s.Step(Vector3.zero);
        Verify(s.DisplacedVolume==0f,"dry body has no buoyancy");
        UnityEngine.Object.DestroyImmediate(go);
    }

    static Trial Simulate(string name,float dt,float duration,float u,float v,bool stop)
    {
        EditorSceneManager.OpenScene(HydroSceneTools.ScenePath);
        var robot=UnityEngine.Object.FindAnyObjectByType<HydroRobot>();robot.keyboard=false;robot.showHud=false;robot.Initialize();
        Physics.SyncTransforms();
        Vector3 start=robot.hull.transform.position;
        var trial=new Trial {name=name,dt=dt,seconds=duration};
        var clock=System.Diagnostics.Stopwatch.StartNew();
        var csv=new StringBuilder("time,u,v,f_hz,A_deg,mean_deg,target_deg,servo_deg,x,y,z,speed,yaw_deg,max_flex_deg,drag_power_W\n");
        float previousTarget=0,previousSpeed=0;
        for(int i=0;i<Mathf.RoundToInt(duration/dt);i++)
        {
            float t=i*dt;robot.u=t<4f || (stop && t>=14f)?0:u;robot.v=t<4f?0:v;
            robot.Step(dt);Physics.Simulate(dt);
            Vector3 p=robot.hull.transform.position;
            float speed=robot.hull.linearVelocity.magnitude;
            float flex=0;
            foreach(var section in robot.sections) flex=Mathf.Max(flex,Mathf.Abs(section.jointPosition[0]*Mathf.Rad2Deg));
            Verify(float.IsFinite(p.x)&&float.IsFinite(p.y)&&float.IsFinite(p.z)&&float.IsFinite(speed),name+" finite dynamics");
            Verify(p.y>-.5f && p.y<.2f && speed<3f,name+" no explosive motion");
            Verify(Mathf.Abs(robot.ServoAngle)<=RobotHydroDefinition.TravelDeg+2f,name+" servo travel");
            Verify(robot.DragPower<=.0001f,name+" dissipative drag");
            Verify(robot.TorqueLimit<=RobotHydroDefinition.StallTorque+.0001f && robot.TorqueLimit>=0f,name+" torque envelope");
            Verify(flex<57f,name+" section travel");
            float targetSpeed=(robot.ServoTarget-previousTarget)/dt;
            trial.maxTargetSpeed=Mathf.Max(trial.maxTargetSpeed,Mathf.Abs(targetSpeed));
            trial.maxTargetAcceleration=Mathf.Max(trial.maxTargetAcceleration,Mathf.Abs(targetSpeed-previousSpeed)/dt);
            previousTarget=robot.ServoTarget;previousSpeed=targetSpeed;
            trial.maxSpeed=Mathf.Max(trial.maxSpeed,speed);trial.maxJointDeg=Mathf.Max(trial.maxJointDeg,flex);
            trial.maxServoDeg=Mathf.Max(trial.maxServoDeg,Mathf.Abs(robot.ServoAngle));trial.maxTrackingDeg=Mathf.Max(trial.maxTrackingDeg,Mathf.Abs(robot.ServoAngle-robot.ServoTarget));
            trial.peakAngularSpeed=Mathf.Max(trial.peakAngularSpeed,robot.hull.angularVelocity.magnitude);
            if(stop && t>duration-2f)trial.lastRestSpeed=Mathf.Max(trial.lastRestSpeed,speed);
            if(i%Mathf.RoundToInt(.05f/dt)==0)
                csv.AppendLine(FormattableString.Invariant($"{t:F3},{robot.u:F2},{robot.v:F2},{robot.Setpoint.frequencyHz:F4},{robot.Setpoint.amplitudeDeg:F3},{robot.Setpoint.meanDeg:F3},{robot.ServoTarget:F3},{robot.ServoAngle:F3},{p.x:F5},{p.y:F5},{p.z:F5},{speed:F5},{Vector3.SignedAngle(Vector3.forward,robot.hull.transform.forward,Vector3.up):F3},{flex:F3},{robot.DragPower:F5}"));
        }
        trial.finalPosition=robot.hull.transform.position;trial.forward=trial.finalPosition.z-start.z;trial.lateral=trial.finalPosition.x-start.x;
        trial.yaw=Vector3.SignedAngle(Vector3.forward,robot.hull.transform.forward,Vector3.up);trial.finalSpeed=robot.hull.linearVelocity.magnitude;trial.wallSeconds=clock.Elapsed.TotalSeconds;
        Verify(trial.maxTargetSpeed<=RobotHydroDefinition.CommandSpeedDeg+.1f,name+" target slew limit");
        Verify(trial.maxTargetAcceleration<=RobotHydroDefinition.CommandAccelerationDeg+5f,name+" target acceleration limit");
        File.WriteAllText("../Documentation/HydroValidation/"+name+".csv",csv.ToString());
        File.WriteAllText("../Documentation/HydroValidation/"+name+".json",JsonUtility.ToJson(trial,true));
        Debug.Log("HYDRO_TRIAL "+JsonUtility.ToJson(trial));
        return trial;
    }
}
