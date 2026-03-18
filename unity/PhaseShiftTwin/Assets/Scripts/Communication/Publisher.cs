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
        [SerializeField] private float _frequency = 10.0f;
        
        protected ROS2.Publisher<T> publisher;
        
        private float _time;
        private float _dt;
        private float _frequency_inv;
        
        public float DT => _frequency_inv;
        public float Time => _time;

        protected override void Awake()
        {
            _dt = 0.0f;
            _frequency_inv = 1.0f / _frequency;
            
            base.Awake();
        }

        public override void Update()
        {
            base.Update();
            
            _dt += UnityEngine.Time.deltaTime;
            if (_dt < _frequency_inv) return;

            _time = UnityEngine.Time.time;
            Publish();

            _dt -= _frequency_inv;
        }

        protected override void OnInitialize()
        {
            base.OnInitialize();
            
            publisher = Node.CreatePublisher<T>(_topicName);
        }

        protected abstract void Publish();

        protected void SetFrequency(float frequency)
        {
            _frequency = frequency;
        }
    }
}