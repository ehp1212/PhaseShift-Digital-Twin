using ROS2;
using UnityEngine;

public class TestSubscriber : MonoBehaviour
{
    private ROS2UnityComponent ros2Unity;
    private ROS2Node ros2Node;
    private ISubscription<phaseshift_interfaces.msg.SystemState> system_sub;
    
    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        ros2Unity = FindFirstObjectByType<ROS2UnityComponent>();
    }

    // Update is called once per frame
    void Update()
    {
        if (ros2Node == null && ros2Unity.Ok())
        {
            ros2Node = ros2Unity.CreateNode("system_listener");
            system_sub = ros2Node.CreateSubscription<phaseshift_interfaces.msg.SystemState>(
                "system_state", msg => Debug.Log("System State: [" + msg.Phase + "]"));
        }
    }
}
