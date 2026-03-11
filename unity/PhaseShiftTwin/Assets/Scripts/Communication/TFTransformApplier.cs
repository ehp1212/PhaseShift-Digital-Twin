using System.Utility;
using UnityEngine;
using Quaternion = geometry_msgs.msg.Quaternion;
using Vector3 = geometry_msgs.msg.Vector3;

namespace Communication
{
    [RequireComponent(typeof(TFSubscriber))]
    public class TFTransformApplier : MonoBehaviour
    {
        private TFSubscriber _subscriber;

        private bool _dirty;
        private UnityEngine.Vector3 _targetTranslation;
        private UnityEngine.Quaternion _targetRotation;

        void Start()
        {
            _subscriber = GetComponent<TFSubscriber>();
            _subscriber.OnTFReceived += SetDirty;
        }

        private void Update()
        {
            // if (!_dirty) return;
            
            Apply(_targetTranslation, _targetRotation);
        }

        private void Apply(UnityEngine.Vector3 translation, UnityEngine.Quaternion quaternion)
        {
            transform.position = translation;
            transform.rotation = quaternion;
            _dirty = false;
        }

        private void SetDirty(Vector3 translation, Quaternion rotation)
        {
            _targetTranslation = TransformUtility.RosToUnityPosition(translation);
            _targetRotation = TransformUtility.RosToUnityRotation(rotation);

            _dirty = true;
        }
    }
}
