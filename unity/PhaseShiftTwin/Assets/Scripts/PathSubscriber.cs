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

public class PathSubscriber : ROS2Subscriber<Path, PathFrame>
{
    private ROS2Node _pathSubNode;
    private ISubscription<Path> _subscription;
    private ROS2System _ros2System;

    private void Awake()
    {
    }

    protected override void SubscribeCallback(Path msg)
    {
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
