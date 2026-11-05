using UnityEngine;
#if ENABLE_INPUT_SYSTEM
using UnityEngine.InputSystem;
#endif
namespace Irys.Hydro
{
    public sealed class HydroCamera : MonoBehaviour
    {
        public Transform target;
        public Vector3 offset = new Vector3(1.05f, .80f, -1.35f);
        public bool overview = true;
        public Vector3 overviewPosition = new Vector3(7.75f,5.85f,-8f);
        public Vector3 overviewTarget = new Vector3(0,-.45f,0);
        void Update()
        {
#if ENABLE_INPUT_SYSTEM
            if(Keyboard.current!=null && Keyboard.current.tabKey.wasPressedThisFrame)overview=!overview;
#else
            if(Input.GetKeyDown(KeyCode.Tab))overview=!overview;
#endif
        }
        void LateUpdate() { RefreshView(); }
        public void RefreshView()
        {
            if (target == null) return;
            GetComponent<Camera>().fieldOfView=overview?65f:43f;
            if(overview)
            {transform.position=overviewPosition;transform.LookAt(overviewTarget);return;}
            Vector3 aim = target.position + Vector3.back * .15f;
            transform.position = aim + offset;
            transform.LookAt(aim);
        }
        void OnGUI()
        {
            if(!overview || target==null)return;
            Vector3 p=GetComponent<Camera>().WorldToScreenPoint(target.position);
            if(p.z>0f)GUI.Box(new Rect(p.x-30f,Screen.height-p.y-28f,60f,24f),"IRYS");
        }
    }
}
