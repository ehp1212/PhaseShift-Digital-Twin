using phaseshift_interfaces.msg;
using ROS2;
using UnityEngine;

namespace System
{
    /// <summary>
    /// Crea
    /// </summary>
    public class ROS2System : ROS2Behaviour
    {
        [Header("State")]
        [SerializeField] private string _nodeName = "phaseshift_unity_system";
        [SerializeField] private string _state_topicName = "phaseshift_state";
        
        protected override string nodeName => _nodeName;
        private ISubscription<SystemState> _systemStateSub;
        
        public static ROS2System Instance { get; private set; }
        public SystemStateMachine SystemState { get; private set; }

        protected override void Awake()
        {
            base.Awake();
            if (Instance != null)
                Destroy(this);

            Instance = this;
            SystemState = new SystemStateMachine();
        }

        protected override void Update()
        {
            base.Update();
            SystemState.OnUpdate();
        }

        protected override void OnRosInitialized()
        {
            Debug.Log($"ROS2System initialized.");
            _systemStateSub = ros2Node.CreateSubscription<SystemState>(_state_topicName, UpdateSystemState);
            SystemState.ApplyPhase(SystemState.Current);
        }

        private void UpdateSystemState(SystemState obj)
        {
            var current = SystemState.Current;
            var next = obj.Phase;
            SystemState.ApplyPhase(next);
        }
    }
}
