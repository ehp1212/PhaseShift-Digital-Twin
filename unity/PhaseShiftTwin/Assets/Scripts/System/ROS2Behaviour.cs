using ROS2;
using UnityEngine;

namespace System
{
    public abstract class ROS2Behaviour : MonoBehaviour
    {
        protected ROS2UnityCore ros2Core;
        protected ROS2Node ros2Node;
        protected virtual string nodeName => "unity_node";
        
        private bool initialized;
        public bool Ok() => ros2Core != null && ros2Core.Ok();

        protected virtual void Awake()
        {
            ros2Core = new ROS2UnityCore();
        }

        protected virtual void Start()
        {

        }

        protected virtual void OnRosInitialized()
        {
            
        }

        protected virtual void Update()
        {
            if (ros2Core == null) return;
            if (!initialized && ros2Core.Ok())
            {
                ros2Node = ros2Core.CreateNode(nodeName);
                initialized = true;
                OnRosInitialized();
            }
        }

        protected virtual void OnDestroy()
        {
        }
    }
}