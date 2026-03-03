using phaseshift_interfaces.msg;
using ROS2;
using Sensor.Lidar._2D;
using UI;
using UnityEngine;
using UnityEngine.Events;

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

        [SerializeField] private Control _control;
        
        protected override string nodeName => _nodeName;
        
        public static ROS2System Instance { get; private set; }
        public SystemStateMachine SystemState { get; private set; }
        public UnityEvent OnInitialize;
        
        private ISubscription<SystemState> _systemStateSub;
        private byte _pendingPhase;

        private ScreenUI ScreenUI { get; set; }
        private ScanRaycastSensor ScanRaycastSensor { get; set; }

        protected override void Awake()
        {
            base.Awake();
            if (Instance != null)
                Destroy(this);

            Instance = this;
            SystemState = new SystemStateMachine(this);
        }

        protected override void Start()
        {
        }

        protected override void Update()
        {
            base.Update();

            // Check state changed and run in main thread
            if (SystemState.Current != _pendingPhase)
            {
                SystemState.ApplyPhase(_pendingPhase);
            }
            
            SystemState.OnUpdate();
        }

        protected override void OnRosInitialized()
        {
            Debug.Log($"ROS2System initialized.");
            OnInitialize?.Invoke();
            
            var context = UnityEngine.Resources.Load<SceneContext>("Context");
            ScreenUI = Instantiate(context.ScreenUI);
            ScanRaycastSensor = Instantiate(context.ScanRaycastSensor, _control.transform);
            ToggleScan(false);
            
            _systemStateSub = ros2Node.CreateSubscription<SystemState>(_state_topicName, SetPendingPhrase);

            var initialPhase = SystemState.Current;
            _pendingPhase = initialPhase;
            SystemState.ApplyPhase(initialPhase, true);
        }
        
        private void SetPendingPhrase(SystemState obj)
        {
            _pendingPhase = obj.Phase;
        }

        public void LogScreenUI(string msg)
        {
            ScreenUI.SetText(msg);
            ScreenUI.Toggle(true);
        }
        
        public void CloseScreenUI()
        {
            ScreenUI.Toggle(false);
        }

        public void ToggleScan(bool toggle)
        {
            ScanRaycastSensor.gameObject.SetActive(toggle);
        }

        public void StartManualDriving(bool start)
        {
            _control.StartDriving(start);
        }
    }
}
