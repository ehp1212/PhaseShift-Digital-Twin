using System;
using UnityEngine;
using ROS2;
using sensor_msgs.msg;
using Communication;
using Time = UnityEngine.Time;

[RequireComponent(typeof(UnityInterfaceCmdSubscriber))]
public class UnityInterfaceStatePublisher : MonoBehaviour
{
    [SerializeField] private string _nodeName = "unity_hardware_interface";
    [SerializeField] private string _topicName = "/unity/state";

    [SerializeField] private float _publishRate = 20f;

    // Husky 기준
    [SerializeField] private float _wheelRadius = 0.1651f;
    [SerializeField] private float _wheelSeparation = 0.55f;

    private ROS2System _ros2System;
    private ROS2Node _node;

    private IPublisher<JointState> _jointPub;
    private UnityInterfaceCmdSubscriber _cmdSub;

    private float _timer;

    // wheel state
    private float _fl_pos, _fr_pos, _rl_pos, _rr_pos;
    private float _fl_vel, _fr_vel, _rl_vel, _rr_vel;

    private void Start()
    {
        _ros2System = ROS2System.Instance;
        _ros2System.OnInitialize.AddListener(Initialize);

        _cmdSub = GetComponent<UnityInterfaceCmdSubscriber>();
    }

    private void Initialize()
    {
        _node = _ros2System.CreateNode(_nodeName);
        _jointPub = _node.CreatePublisher<JointState>(_topicName);
    }

    private void Update()
    {
        if (_jointPub == null) return;

        float dt = Time.deltaTime;
        
        // ------------------------
        // cmd_vel → wheel velocity
        // ------------------------
        float v = (_cmdSub != null) ? _cmdSub.CurrentCmd.x : 0f;
        float w = (_cmdSub != null) ? _cmdSub.CurrentCmd.y : 0f;

        float left_vel = v - (w * _wheelSeparation / 2f);
        float right_vel = v + (w * _wheelSeparation / 2f);

        // rad/s
        _fl_vel = left_vel / _wheelRadius;
        _rl_vel = left_vel / _wheelRadius;
        _fr_vel = right_vel / _wheelRadius;
        _rr_vel = right_vel / _wheelRadius;

        // ------------------------
        // integrate position
        // ------------------------
        _fl_pos += _fl_vel * dt;
        _rl_pos += _rl_vel * dt;
        _fr_pos += _fr_vel * dt;
        _rr_pos += _rr_vel * dt;

        // ------------------------
        // publish rate control
        // ------------------------
        _timer += dt;
        if (_timer >= 1f / _publishRate)
        {
            Publish();
            _timer = 0f;
        }
    }

    private void Publish()
    {
        var msg = new JointState();

        // ------------------------
        // Header
        // ------------------------
        msg.Header = new std_msgs.msg.Header();

        // ------------------------
        // Joint names (URDF랑 반드시 동일!)
        // ------------------------
        msg.Name = new string[]
        {
            "front_left_wheel_joint",
            "rear_left_wheel_joint",
            "front_right_wheel_joint",
            "rear_right_wheel_joint"
        };

        // ------------------------
        // Position (rad)
        // ------------------------
        msg.Position = new double[]
        {
            _fl_pos,
            _rl_pos,
            _fr_pos,
            _rr_pos
        };

        // ------------------------
        // Velocity (rad/s)
        // ------------------------
        msg.Velocity = new double[]
        {
            _fl_vel,
            _rl_vel,
            _fr_vel,
            _rr_vel
        };

        Debug.Log($"v: {_cmdSub.CurrentCmd.x}, w: {_cmdSub.CurrentCmd.y}");
        UpdateTimeStamp(ref msg);
        _jointPub.Publish(msg);
    }
    
    private void UpdateTimeStamp(ref JointState msg)
    {
        var clockMsg = new rosgraph_msgs.msg.Clock();
        _node.clock.UpdateClockMessage(ref clockMsg);

        msg.UpdateHeaderTime(clockMsg.Clock_.Sec, clockMsg.Clock_.Nanosec);
    }
}