using System;
using geometry_msgs.msg;
using ROS2;
using UnityEngine;

public class Control : MonoBehaviour
{
    [Header("Input Node")]
    [SerializeField] private string _nodeName = "unity_manual_input";
    [SerializeField] private string _topicName = "/cmd_vel";
    
    [Space]
    [Header("Motion Limits")]
    [SerializeField] private float maxLinearVelocity = 1.0f;   // m/s
    [SerializeField] private float maxAngularVelocity = 1.0f;  // rad/s
    
    public string NodeName => _nodeName;
    public string TopicName => _topicName;
    public float MaxLinearVelocity => maxLinearVelocity;
    public float MaxAngularVelocity => maxAngularVelocity;

    public InputRouter InputRouter { get; set; }

    private void Start()
    {
        InputRouter = new InputRouter(this);
    }

    private void Update()
    {
        InputRouter.ProcessInput();
        InputRouter.PublishInput();
    }
}

public class InputRouter
{
    private readonly Control _control;
    
    private ROS2System _ros2System;
    private float _linearInput;
    private float _angularInput;
    private ROS2Node _node;
    private Publisher<Twist> _cmdPublisher;
    private Twist _twistMsg;

    public InputRouter(Control control)
    {
        _control = control;
        
        _ros2System = ROS2System.Instance;
        _node = _ros2System.CreateNode(_control.NodeName);
        _cmdPublisher = _node.CreatePublisher<Twist>(_control.TopicName);
        _twistMsg = new Twist();
    }

    public void ProcessInput()
    {
        _linearInput = Input.GetAxis("Vertical");   // W/S
        _angularInput = Input.GetAxis("Horizontal"); // A/D
    }

    public void PublishInput()
    {
        _twistMsg.Linear.X = _linearInput * _control.MaxLinearVelocity;
        _twistMsg.Linear.Y = 0.0;
        _twistMsg.Linear.Z = 0.0;

        _twistMsg.Angular.X = 0.0;
        _twistMsg.Angular.Y = 0.0;
        _twistMsg.Angular.Z = _angularInput * _control.MaxAngularVelocity;

        _cmdPublisher.Publish(_twistMsg);
    }
}
