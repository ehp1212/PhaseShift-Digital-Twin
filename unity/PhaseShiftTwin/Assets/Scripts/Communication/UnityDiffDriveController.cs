using UnityEngine;
using Quaternion = geometry_msgs.msg.Quaternion;
using Vector3 = geometry_msgs.msg.Vector3;

namespace Communication
{
    [RequireComponent(typeof(UnityInterfaceCmdSubscriber))]
    public class UnityDiffDriveController : MonoBehaviour
    {
        private UnityInterfaceCmdSubscriber _cmd;

        [SerializeField] private float _maxLinearSpeed = 2.0f;
        [SerializeField] private float _maxAngularSpeed = 2.0f;

        private float _v; // linear
        private float _w; // angular

        private float _theta; // heading

        private void Start()
        {
            _cmd = GetComponent<UnityInterfaceCmdSubscriber>();

            _cmd.OnCmdReceived += OnCmd;

            _theta = transform.eulerAngles.y * Mathf.Deg2Rad;
        }

        private void OnCmd(geometry_msgs.msg.Twist msg)
        {
            _v = Mathf.Clamp((float)msg.linear.x, -_maxLinearSpeed, _maxLinearSpeed);
            _w = Mathf.Clamp((float)msg.angular.z, -_maxAngularSpeed, _maxAngularSpeed);
        }

        private void Update()
        {
            float dt = Time.deltaTime;

            // ------------------------
            // Heading update
            // ------------------------
            _theta += _w * dt;

            // ------------------------
            // Position update (ROS 기준)
            // ------------------------
            float dx = _v * Mathf.Cos(_theta) * dt;
            float dy = _v * Mathf.Sin(_theta) * dt;

            // ------------------------
            // Unity 좌표 변환
            // ------------------------
            Vector3 delta = new Vector3(
                -dy,   // Unity X
                0,
                dx     // Unity Z
            );

            transform.position += delta;

            // ------------------------
            // Rotation update
            // ------------------------
            transform.rotation = Quaternion.Euler(
                0,
                _theta * Mathf.Rad2Deg,
                0
            );
        }
    }
}