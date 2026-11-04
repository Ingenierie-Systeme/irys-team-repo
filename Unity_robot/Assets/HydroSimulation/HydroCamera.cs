using UnityEngine;
namespace Irys.Hydro
{
    public sealed class HydroCamera : MonoBehaviour
    {
        public Transform target;
        public Vector3 offset = new Vector3(1.05f, .80f, -1.35f);
        void LateUpdate() { RefreshView(); }
        public void RefreshView()
        {
            if (target == null) return;
            Vector3 aim = target.position + Vector3.back * .15f;
            transform.position = aim + offset;
            transform.LookAt(aim);
        }
    }
}
