using System.Linq;
using ROS2;
using sensor_msgs.msg;
using UnityEngine;

namespace Communication.Camera
{
    public abstract class CameraPublisher<T> : Publisher<T> where T : Message, new()
    {
        private IPublisher<CameraInfo> _cameraInfoPublisher;
        
        protected RenderTexture sourceTexture;
        protected UnityEngine.Camera renderCamera;
        protected int width;
        protected int height;
        private CameraInfo _infoMsg;

        protected override void Awake()
        {
            base.Awake();
            renderCamera = GetComponent<UnityEngine.Camera>();
        }

        protected override void Start()
        {
            base.Start();
            
            width = sourceTexture.width;
            height = sourceTexture.height;
        }

        protected override void OnInitialize()
        {
            base.OnInitialize();
            
            var cameraSpace = TopicName.Split('/');

            // 마지막이 compressed면 한 단계 더 올라가야 함
            if (cameraSpace[^1] == "compressed")
            {
                cameraSpace[^2] = "camera_info";
                cameraSpace = cameraSpace.Take(cameraSpace.Length - 1).ToArray();
            }
            else
            {
                cameraSpace[^1] = "camera_info";
            }
         
            var cameraInfoTopic = string.Join("/", cameraSpace);
            _cameraInfoPublisher = Node.CreatePublisher<CameraInfo>(cameraInfoTopic);

            SetupIntrinsics();
        }

        private void SetupIntrinsics()
        {
            var msg = new CameraInfo();

            msg.Width = (uint)width;
            msg.Height = (uint)height;

            // intrinsics
            var fy = height / (2f * Mathf.Tan(renderCamera.fieldOfView * 0.5f * Mathf.Deg2Rad));
            var fx = fy * ((float)width / height);

            var cx = width / 2f;
            var cy = height / 2f;

            msg.K[0] = fx;
            msg.K[1] = 0.0;
            msg.K[2] = cx;
            msg.K[3] = 0.0;
            msg.K[4] = fy;
            msg.K[5] = cy;
            msg.K[6] = 0.0;
            msg.K[7] = 0.0;
            msg.K[8] = 1.0;

            _infoMsg = msg;
        }
        
        protected override void Publish()
        {
            UpdateTimeStamp(ref _infoMsg);
            _cameraInfoPublisher.Publish(_infoMsg);
        }
        
        private void UpdateTimeStamp(ref CameraInfo cameraInfo)
        {
            var clockMsg = new rosgraph_msgs.msg.Clock();
            Node.clock.UpdateClockMessage(ref clockMsg);

            cameraInfo.UpdateHeaderTime(clockMsg.Clock_.Sec, clockMsg.Clock_.Nanosec);
        }
    }
}