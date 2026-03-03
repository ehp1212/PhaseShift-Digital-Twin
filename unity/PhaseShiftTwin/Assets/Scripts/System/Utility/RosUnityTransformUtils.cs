using UnityEngine;

namespace System.Utility
{
    public static class TransformUtility
    {
        // ---------------------------
        // ROS Quaternion → Yaw 추출
        // ---------------------------
        private static float GetYawFromRos(geometry_msgs.msg.Quaternion q)
        {
            var siny_cosp = 2f * (q.W * q.Z + q.X * q.Y);
            var cosy_cosp = 1f - 2f * (q.Y * q.Y + q.Z * q.Z);
            return Mathf.Atan2((float)siny_cosp, (float)cosy_cosp);
        }

        // ---------------------------
        // ROS → Unity Rotation
        // ---------------------------
        public static Quaternion RosToUnityRotation(geometry_msgs.msg.Quaternion q)
        {
            float yaw = Mathf.Atan2(
                2.0f * (float)(q.W * q.Z),
                1.0f - 2.0f * (float)(q.Z * q.Z)
            );

            float yawDeg = yaw * Mathf.Rad2Deg;

            // ★ 이게 핵심: Position 매핑이 x=-y이므로 yaw도 -로 맞춰야 함
            yawDeg = -yawDeg;

            return Quaternion.Euler(0f, yawDeg, 0f);
        }

        // ---------------------------
        // ROS → Unity Position
        // ---------------------------
        public static Vector3 RosToUnityPosition(geometry_msgs.msg.Vector3 ros)
        {
            return new Vector3(
                -(float)ros.Y,
                0,
                (float)ros.X
            );
        }
        
        public static Vector3 AngleToUnityDirection(float angleRad)
        {
            return new Vector3(
                -Mathf.Sin(angleRad),
                0f,
                Mathf.Cos(angleRad)
            );
        }
    }
}