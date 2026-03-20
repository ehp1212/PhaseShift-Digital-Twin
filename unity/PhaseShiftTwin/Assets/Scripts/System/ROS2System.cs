using System.Collections.Generic;
using phaseshift_interfaces.msg;
using ROS2;
using Sensor.Lidar._2D;
using Sensor.Visualizer;
using UI;
using UnityEngine;
using UnityEngine.Events;

namespace System
{
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
        public SLAMUI SLAMUI { get; set; }
        public bool IsOk => Ok();
        
        private ScanRaycastSensor ScanRaycastSensor { get; set; }
        private SLAMGeometryMapVisualizer SLAMGeometryMapVisualizer { get; set; }
        public ROS2ServiceController ROS2ServiceController { get; set; }

        public delegate void OnPhaseChangedHandler(byte previousPhase, byte newPhase);
        public OnPhaseChangedHandler OnPhaseChanged;
        
        private Dictionary<string, ROS2Node> _createdNodes = new();
        
        protected override void Awake()
        {
            base.Awake();
            if (Instance != null)
                Destroy(this);

            Instance = this;
            SystemState = new SystemStateMachine(this);
            ROS2ServiceController = new ROS2ServiceController(this);
            OnInitialize.AddListener(ROS2ServiceController.Initialize);
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
        
        public ROS2Node CreateNode(string nName)
        {
            if (_createdNodes.TryGetValue(nName, out ROS2Node existingNode)) 
                return existingNode;
            
            var node = ros2Core.CreateNode(nName);
            _createdNodes.Add(nName, node);
            return node;
        }

        public void RemoveNode(ROS2Node node)
        {
            if (_createdNodes.TryGetValue(name, out ROS2Node _))
                ros2Core.RemoveNode(node);
        }

        protected override void OnRosInitialized()
        {
            Debug.Log($"ROS2System initialized.");
            OnInitialize?.Invoke();
            
            var context = UnityEngine.Resources.Load<SceneContext>("Context");
            ScreenUI = Instantiate(context.ScreenUI);
            ScanRaycastSensor = Instantiate(context.ScanRaycastSensor, _control.transform);
            ToggleScan(false);
            
            SLAMUI = Instantiate(context.SlamUI);
            SLAMUI.Toggle(false);
            
            SLAMGeometryMapVisualizer = FindFirstObjectByType<SLAMGeometryMapVisualizer>();
            SLAMGeometryMapVisualizer.Initialize(ScanRaycastSensor);
            SLAMGeometryMapVisualizer.Show(false);
            
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
            Debug.Log($"TOGGLE SCAN: {toggle}");
            ScanRaycastSensor.gameObject.SetActive(toggle);
        }

        public void StartManualDriving(bool start)
        {
            _control.StartDriving(start);
        }

        public void EnterSlamPhase(bool enter)
        {
            SLAMUI.Toggle(enter);
            SLAMGeometryMapVisualizer.Show(enter);
        }

        public bool TryGetNode(string nName, out ROS2Node node)
        {
            return _createdNodes.TryGetValue(nName, out node);
        }
    }
}
