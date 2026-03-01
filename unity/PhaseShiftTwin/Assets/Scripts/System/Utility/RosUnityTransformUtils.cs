using UnityEngine;

namespace System.Utility
{
    public static class TransformUtility
    {
        /*
         * ==========================================
         * POSITION
         * ==========================================
         */

        // Unity → ROS
        public static Vector3 UnityToRosPosition(Vector3 unityPosition)
        {
            return new Vector3(
                unityPosition.z,
                -unityPosition.x,
                unityPosition.y
            );
        }

        // ROS → Unity
        public static Vector3 RosToUnityPosition(Vector3 rosPosition)
        {
            return new Vector3(
                -rosPosition.y,
                rosPosition.z,
                rosPosition.x
            );
        }

        /*
         * ==========================================
         * DIRECTION
         * (Same math as position, but semantic clarity)
         * ==========================================
         */

        public static Vector3 UnityToRosDirection(Vector3 unityDirection)
        {
            return UnityToRosPosition(unityDirection);
        }

        public static Vector3 RosToUnityDirection(Vector3 rosDirection)
        {
            return RosToUnityPosition(rosDirection);
        }

        /*
         * ==========================================
         * ROTATION (Quaternion)
         * ==========================================
         */

        public static Quaternion UnityToRosRotation(Quaternion unityQuat)
        {
            return new Quaternion(
                unityQuat.z,
                -unityQuat.x,
                unityQuat.y,
                -unityQuat.w
            );
        }

        public static Quaternion RosToUnityRotation(Quaternion rosQuat)
        {
            return new Quaternion(
                -rosQuat.y,
                rosQuat.z,
                rosQuat.x,
                -rosQuat.w
            );
        }

        /*
         * ==========================================
         * EULER (Optional)
         * ==========================================
         */

        public static Vector3 UnityToRosEuler(Vector3 unityEulerDeg)
        {
            Quaternion q = Quaternion.Euler(unityEulerDeg);
            return UnityToRosRotation(q).eulerAngles;
        }

        public static Vector3 RosToUnityEuler(Vector3 rosEulerDeg)
        {
            Quaternion q = Quaternion.Euler(rosEulerDeg);
            return RosToUnityRotation(q).eulerAngles;
        }
        
        public static Vector3 AngleToUnityDirection(float angleRad)
        {
            return new Vector3(
                Mathf.Sin(angleRad),
                0f,
                Mathf.Cos(angleRad)
            );
        }
    }
}