using System;
using Communication.Thread;
using phaseshift_interfaces.msg;
using UnityEngine;
using Pose = geometry_msgs.msg.Pose;

namespace Communication
{
    public struct DetectionObjectFrame : IThreadFrame
    {
        public string Class_Id;
        public float Confidence;
        public Pose Pose;
    }
    
    public struct DetectionObjectArrayFrame : IThreadFrame
    {
        public DetectionObjectFrame[] DetectionObjects;
    }
    
    public class DetectionObjectSubscriber : ROS2Subscriber<DetectedObjectArray, DetectionObjectArrayFrame>, IROS2Interface
    {
        public bool Active { get; set; }
        public void Toggle(bool active)
        {
            Active = active;
        }
        
        // Start is called once before the first execution of Update after the MonoBehaviour is created
        protected override void SubscribeCallback(DetectedObjectArray msg)
        {
            if (!Active) return;
            
            var count = msg.Objects.Length;
            var result = new DetectionObjectArrayFrame();
            result.DetectionObjects = new DetectionObjectFrame[count];
            
            for (int i = 0; i < count; i++)
            {
                var obj = msg.Objects[i];

                var frame = new DetectionObjectFrame()
                {
                    Class_Id = obj.Class_id,
                    Confidence = obj.Confidence,
                    Pose = obj.Pose
                };

                result.DetectionObjects[i] = frame;
            }

            dispatcher.Push(result);
        }
    }
}
