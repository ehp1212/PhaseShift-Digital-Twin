using System;
using ROS2;
using UnityEngine;

namespace Communication
{
    public abstract class Ros2Node : MonoBehaviour
    {
        [SerializeField] private string _nodeName;
        [SerializeField] private string _frameId;
     
        protected ROS2System Ros2System;
        protected ROS2Node Node;
        protected string FrameId => _frameId;

        protected virtual void Awake()
        {
        }
        
        protected virtual void Start()
        {
            Ros2System = ROS2System.Instance;
            Ros2System.OnInitialize.AddListener(OnInitialize);
        }

        public virtual void Update()
        {
            if (Ros2System == null)
            {
                Debug.Log("Late initialized");
                Ros2System = ROS2System.Instance;
                OnInitialize();
            }
        }

        protected virtual void OnInitialize()
        {
            Node = Ros2System.CreateNode(_nodeName);
        }
    }
}