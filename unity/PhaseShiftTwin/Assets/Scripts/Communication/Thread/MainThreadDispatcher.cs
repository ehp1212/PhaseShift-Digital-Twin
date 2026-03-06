using System.Threading;
using Sensor.Visualizer;
using UnityEngine;

namespace Communication.Thread
{
    public class MainThreadDispatcher<T>
    {
        private T _latest;
        private int _hasNewFrame; // 0 -> No, 1 -> Yes

        /// <summary>
        /// Called from non-main thread to push main thread (e.g., ROS Callbacks)
        /// </summary>
        /// <param name="frame"></param>
        public void Push(T frame)
        {
            _latest = frame;
            Interlocked.Exchange(ref _hasNewFrame, 1);
        }
        
        /// <summary>
        /// Called from Unity main thread (Update).
        /// Returns true if a new frame was available.
        /// </summary>
        public bool TryDequeueLatest(out T frame)
        {
            // atomically check & reset flag
            if (Interlocked.Exchange(ref _hasNewFrame, 0) == 1)
            {
                frame = _latest;
                return true;
            }

            frame = default;
            return false;
        }
        
        /// <summary>
        /// Returns whether a new frame is waiting (optional utility).
        /// </summary>
        public bool HasPendingFrame => _hasNewFrame == 1;
    }

    public interface IThreadFrame
    {
        
    }

    public struct OccupancyGridFrame : IThreadFrame
    {
        public int Width;
        public int Height;
        public float Resolution;

        public Vector3 OriginRos;   // map frame origin
        public Quaternion OriginRotationRos;
        
        public sbyte[] Data;        // immutable after creation
    }
    
    public struct PointCloudFrame
    {
        public PointXYZRGB[] Points;
        public int Count;
    }
}
