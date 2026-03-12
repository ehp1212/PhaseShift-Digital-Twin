using System;
using Communication.Thread;
using ROS2;
using UnityEngine;

namespace Communication
{
    public interface IROS2Subscription { }

    /// <summary>
    /// Subscriber class receiving data from ROS2
    /// Push to main thread
    /// </summary>
    /// <typeparam name="TMessage"></typeparam>
    /// <typeparam name="TFrame"></typeparam>
    public abstract class ROS2Subscriber<TMessage, TFrame> : MonoBehaviour, IROS2Subscription 
        where TMessage : Message, new()
        where TFrame : IThreadFrame
    {
        [SerializeField] protected string nodeName;
        [SerializeField] protected string topicName;

        protected ROS2System ros2System;
        protected ROS2Node node;
        protected ISubscription<TMessage> subscription;

        public MainThreadDispatcher<TFrame> dispatcher = new();
        
        protected abstract void SubscribeCallback(TMessage msg);

        protected virtual void Start()
        {
            ros2System = ROS2System.Instance;
            ros2System.OnInitialize.AddListener(OnInitialize);
        }

        protected virtual void OnInitialize()
        {
            if (string.IsNullOrWhiteSpace(topicName) || string.IsNullOrWhiteSpace(nodeName))
            {
                Debug.LogError($"Failed to create.");
                return;
            }

            if (!ros2System.TryGetNode(nodeName, out node))
                node = ros2System.CreateNode(nodeName);
            
            subscription = node.CreateSubscription<TMessage>(topicName,
                SubscribeCallback
            );
        }

        private void OnDestroy()
        {
            ros2System?.OnInitialize.RemoveListener(OnInitialize);
        }
    }
}
