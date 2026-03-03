using System;
using ROS2;
using tf2_msgs.msg;
using UnityEngine;
using Quaternion = geometry_msgs.msg.Quaternion;
using Vector3 = geometry_msgs.msg.Vector3;

namespace Communication
{
    /*Subscribe TF and extract transform*/
    public class TFSubscriber : MonoBehaviour
    {
        [SerializeField] private string parentFrame = "map";
        [SerializeField] private string childFrame = "base_footprint";
        [Space]
        [SerializeField] private string _nodeName = "unity_tf_subscriber";
        [SerializeField] private string _topicName = "tf";
        
        private ROS2System _ros2System;
        private ROS2Node _node;
        private ISubscription<TFMessage> _tfSub;
        
        public Vector3 Translation { get; set; }
        public Quaternion Rotation { get; set; }
        
        public Action<Vector3, Quaternion> OnTFReceived;

        private void Start()
        {
            _ros2System = ROS2System.Instance;
            _ros2System.OnInitialize.AddListener(SetUp);
        }

        private void SetUp()
        {
            _node = _ros2System.CreateNode(_nodeName);
            _tfSub = _node.CreateSubscription<TFMessage>(_topicName, ProcessTF);
        }

        private void ProcessTF(TFMessage msg)
        {
            foreach (var transformStamped in msg.Transforms)
            {
                if (transformStamped.Header.Frame_id != parentFrame || transformStamped.Child_frame_id != childFrame) continue;

                Translation = transformStamped.Transform.Translation;
                Rotation = transformStamped.Transform.Rotation;
                
                OnTFReceived?.Invoke(Translation, Rotation);
            }
        }
    }
}
