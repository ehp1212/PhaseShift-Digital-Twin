using sensor_msgs.msg;
using std_msgs.msg;
using UnityEngine;
using UnityEngine.Rendering;
using UnitySensors.Sensor.Camera;

namespace Communication.Camera
{
    [RequireComponent(typeof(HDRPRGBCompressedCamera))]
    public class RGBCompressedImagePublisher : CameraPublisher<CompressedImage>
    {
        [SerializeField] private HDRPRGBCompressedCamera compressedCameraSensor;
        
        private Texture2D _compressedTexture;
        private byte[] _jpgBuffer;
        private CompressedImage _msg;

        protected override void Awake()
        {
            base.Awake();
        }

        protected override void Start()
        {
            // Set source data before setting camera info node in parent
            sourceTexture = compressedCameraSensor.RenderTexture;
            if (sourceTexture == null)
                Debug.LogError($"{nameof(RGBCompressedImagePublisher)} requires a {nameof(sourceTexture)}");
            
            base.Start();
        }

        protected override void OnInitialize()
        {
            base.OnInitialize();
            
            _compressedTexture = new Texture2D(width, height, TextureFormat.RGBA32, false);
            
            _msg = new CompressedImage();
            _msg.Header = new Header();
            _msg.Header.Frame_id = FrameId;

            _msg.Format = "jpeg";
        }

        /* Publish data by frequency not onSensorUpdated in sensor */
        protected override void Publish()
        {
            if (compressedCameraSensor == null)
            {
                Debug.LogError($"{nameof(RGBCompressedImagePublisher)} requires a {nameof(RGBCameraSensor)}");
                return;
            }
            
            AsyncGPUReadback.Request(sourceTexture, 0, OnReadback);
        }

        private void OnReadback(AsyncGPUReadbackRequest request)
        {
            if (request.hasError || _compressedTexture == null) 
                return;
            
            var raw = request.GetData<byte>();
            
            // Gpu -> Texture2D
            _compressedTexture.LoadRawTextureData(raw);
            _compressedTexture.Apply();

            _jpgBuffer = _compressedTexture.EncodeToJPG();
            PublishCompressed(_jpgBuffer);
        }

        private void PublishCompressed(byte[] jpgBuffer)
        {
            if (!Ros2System.IsOk) return;
            
            _msg.Data = jpgBuffer;

            UpdateTimeStamp(ref _msg);
            publisher.Publish(_msg);

            // Publish Camera info
            base.Publish();
        }
        
        private void UpdateTimeStamp(ref CompressedImage compressedImage)
        {
            var clockMsg = new rosgraph_msgs.msg.Clock();
            Node.clock.UpdateClockMessage(ref clockMsg);

            compressedImage.UpdateHeaderTime(clockMsg.Clock_.Sec, clockMsg.Clock_.Nanosec);
        }
    }
}
