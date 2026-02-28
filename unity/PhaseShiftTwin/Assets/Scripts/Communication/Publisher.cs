using ROS2;
using UnityEngine;

namespace Communication
{
    /// <summary>
    /// Base class for publishing data to ros2 with ros2forunity
    /// Do not use unity callbacks for ros2forunity
    /// </summary>
    public abstract class Publisher<T> : Ros2Node where T : Message, new()
    {
        [SerializeField] private string _topicName;
        
        protected ROS2.Publisher<T> publisher;

        protected override void OnInitialize()
        {
            base.OnInitialize();
            
            publisher = Node.CreatePublisher<T>(_topicName);
        }
    }
}