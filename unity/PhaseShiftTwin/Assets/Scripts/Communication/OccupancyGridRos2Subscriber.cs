using Communication.Thread;
using nav_msgs.msg;
using UnityEngine;

namespace Communication
{
    public class OccupancyGridRos2Subscriber : ROS2Subscriber<OccupancyGrid, OccupancyGridFrame>
    {
        /// <summary>
        /// Get OccupancyGrid Message and convert it to OccupancyGridFrame then push to main thread 
        /// </summary>
        /// <param name="msg"></param>
        protected override void SubscribeCallback(OccupancyGrid msg)
        {
            var origin = msg.Info.Origin.Position;
            var frame = new OccupancyGridFrame
            {
                Width = (int)msg.Info.Width,
                Height = (int)msg.Info.Height,
                Resolution = msg.Info.Resolution,
                Data = (sbyte[])msg.Data.Clone(),
                
                OriginRos = new Vector3(
                    (float)origin.X,
                    0f,
                    (float)origin.Y
                ),
            };
            
            // Push to main thread
            dispatcher.Push(frame);
        }
    }
}
