using System;
using phaseshift_interfaces.msg;
using ROS2;
using UnityEngine;

public class FeedbackListener : MonoBehaviour
{
    [SerializeField] private string _nodeName = "unity_feedback_listener";
    [SerializeField] private string _topicName = "/system/navigation_feedback";

    private ROS2Node _feedbackSubNode;
    private ISubscription<NavigationFeedback> _subscription;
    private ROS2System _ros2System;
    
    void Start()
    {
        _ros2System = ROS2System.Instance;
        _ros2System.OnInitialize.AddListener(SetUp);
    }

    private void SetUp()
    {
        _feedbackSubNode = _ros2System.CreateNode(_nodeName);
        _subscription = _feedbackSubNode.CreateSubscription<NavigationFeedback>(_topicName, FeedbackCallback);
    }

    private void FeedbackCallback(NavigationFeedback msg)
    {
        if (_ros2System.SystemState.Current != SystemPhases.PHASE_NAV_EXECUTING) return;
        return;
        Debug.Log($"Distance remaining {msg.Distance_remaining}");
        Debug.Log($"Navigation time {msg.Navigation_time}");
        Debug.Log($"ETR {msg.Estimated_time_remaining}");
    }
}
