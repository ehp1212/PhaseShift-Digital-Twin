using System.Linq;
using ROS2;
using sensor_msgs.msg;

namespace Communication.Camera
{
    public abstract class CameraPublisher<T> : Publisher<T> where T : Message, new()
    {
        private IPublisher<CameraInfo> _cameraInfoPublisher;
        
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
        }

        protected override void Publish()
        {
            _cameraInfoPublisher.Publish(new CameraInfo());
        }
    }
}