using System;
using Communication;
using Communication.Thread;
using nav_msgs.msg;
using ROS2;
using UnityEngine;

public struct PathFrame : IThreadFrame 
{
    public Vector3[] PathPoints;
}

public class PathSubscriber : ROS2Subscriber<Path, PathFrame>, IROS2Interface
{
    private ROS2Node _pathSubNode;
    private ISubscription<Path> _subscription;
    private ROS2System _ros2System;
    
    public bool Active { get; set; }
    public void Toggle(bool active)
    {
        Active = active;
    }

    protected override void SubscribeCallback(Path msg)
    {
        if (!Active) return;
        var count = msg.Poses.Length;
        var points = new Vector3[count];
        for (int i = 0; i < count; i++)
        {
            var pose = msg.Poses[i].Pose.Position;
            var rosX = (float)pose.X;
            var rosY = (float)pose.Y;

            // ROS → Unity
            points[i] = new Vector3(
                -rosY,
                0.05f,
                rosX
            );
            
        }
        
        dispatcher.Push(new PathFrame { PathPoints = points });
    }
}
