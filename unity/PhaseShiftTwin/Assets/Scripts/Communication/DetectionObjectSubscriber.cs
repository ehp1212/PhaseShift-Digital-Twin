using System;
using Communication.Thread;
using phaseshift_interfaces.msg;
using UnityEngine;
using Pose = geometry_msgs.msg.Pose;
using Vector3 = geometry_msgs.msg.Vector3;

namespace Communication
{
    public struct TrackedObjectFrame : IThreadFrame
    {
        public int Id;
        public string Class_Id;
        public Pose Pose;
        
        public Vector3 Velocity;
        public bool Is_dynamic;
        public float Motion_Confidence;
        public float Last_seen_timestamp;
    }
    
    public struct TrackedObjectArrayFrame : IThreadFrame
    {
        public TrackedObjectFrame[] TrackedObjects;
    }
    
    public class DetectionObjectSubscriber : ROS2Subscriber<TrackedObjectArray, TrackedObjectArrayFrame>, IROS2Interface
    {
        public bool Active { get; set; }
        public void Toggle(bool active)
        {
            Active = active;
        }
        
        // Start is called once before the first execution of Update after the MonoBehaviour is created
        protected override void SubscribeCallback(TrackedObjectArray msg)
        {
            if (!Active) return;
            
            var count = msg.Objects.Length;
            var result = new TrackedObjectArrayFrame();
            result.TrackedObjects = new TrackedObjectFrame[count];
            
            for (int i = 0; i < count; i++)
            {
                var obj = msg.Objects[i];

                var frame = new TrackedObjectFrame()
                {
                    Id = obj.Id,
                    Class_Id = obj.Class_id,
                    Pose = obj.Pose,
                    
                    Velocity = obj.Velocity,
                    Is_dynamic = obj.Is_dynamic,
                    Motion_Confidence = obj.Motion_confidence,
                    Last_seen_timestamp = obj.Last_seen_time
                };

                result.TrackedObjects[i] = frame;
            }

            dispatcher.Push(result);
        }
    }
}
