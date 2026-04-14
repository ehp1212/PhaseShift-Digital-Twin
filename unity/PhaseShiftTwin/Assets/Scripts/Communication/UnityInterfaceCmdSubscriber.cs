using System;
using geometry_msgs.msg;
using ROS2;
using UnityEngine;

namespace Communication
{
    public class UnityInterfaceCmdSubscriber : MonoBehaviour
    {
        [SerializeField] private string _nodeName = "unity_hardware_interface";
        [SerializeField] private string _topicName = "/diff_drive_controller/cmd_vel_unstamped";
        
        private ROS2System _ros2System;
        private ROS2Node _node;

        private ISubscription<Twist> _cmdSub;
        private Twist _cachedMsg;

        public Vector2 CurrentCmd { get; private set; } // (v, w)
        public Action<Twist> OnCmdReceived;
        
        private void Start()
        {
            _ros2System = ROS2System.Instance;
            _ros2System.OnInitialize.AddListener(Initialize);
        }

        private void Initialize()
        {
            _node = _ros2System.CreateNode(_nodeName);
            _cmdSub = _node.CreateSubscription<Twist>(_topicName,
                SubscribeCallback);
        }

        private void SubscribeCallback(Twist msg)
        {
            _cachedMsg = msg;

            // v, w 저장
            CurrentCmd = new Vector2(
                (float)msg.Linear.X,
                (float)msg.Angular.Z
            );

            OnCmdReceived?.Invoke(msg);
        }
    }
}