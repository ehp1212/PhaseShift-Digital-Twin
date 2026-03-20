using ROS2;
using sensor_msgs.msg;
using std_msgs.msg;
using Unity.Collections;
using UnityEngine;
using UnityEngine.Rendering;

namespace Communication.Camera
{
    public class HDRPDepthCameraPublisher : CameraPublisher<Image>
    {
        [SerializeField] private RenderTexture _depthRenderTexture;
        [SerializeField] private ComputeShader _depthToPointShader;
        
        [SerializeField] protected float _minRange = 0.05f;
        [SerializeField] protected float _maxRange = 100.0f;
        
        [Header("Point Cloud")]
        [SerializeField] private bool _publishPointCloud = true;
        [SerializeField] private string _pcTopicName = "/camera/depth/points";

        private IPublisher<PointCloud2> _pointCloudPublisher;
        private ComputeBuffer _pointBuffer;
        private ComputeBuffer _depthBuffer;
        private int _kernel;
        private Image _msg;
        private byte[] _bytes;
        
        protected override void Awake()
        {
            base.Awake();
        }

        protected override void Start()
        {
            // Set source data before setting camera info node in parent
            sourceTexture = _depthRenderTexture;
            if (sourceTexture == null)
                Debug.LogError($"{nameof(RGBCompressedImagePublisher)} requires a {nameof(sourceTexture)}");
            
            base.Start();

            renderCamera.nearClipPlane = _minRange;
            renderCamera.farClipPlane = _maxRange;
            
            _pointBuffer = new ComputeBuffer(sourceTexture.width * sourceTexture.height, sizeof(float) * 3);
            _depthBuffer = new ComputeBuffer(sourceTexture.width * sourceTexture.height, sizeof(float));
            _kernel = _depthToPointShader.FindKernel("CSMain");
            
            _depthToPointShader.SetTexture(_kernel, "depthTexture", sourceTexture);
            _depthToPointShader.SetBuffer(_kernel, "pointsBuffer", _pointBuffer);
            _depthToPointShader.SetBuffer(_kernel, "flippedDepthBuffer", _depthBuffer);
            
            _depthToPointShader.SetInt("width", width);
            _depthToPointShader.SetInt("height", height);
            
            SetupIntrinsics();
        }

        protected override void OnInitialize()
        {
            base.OnInitialize();
            
            // Point Cloud
            if (!_publishPointCloud) return;
            
            _pointCloudPublisher = Node.CreatePublisher<PointCloud2>(_pcTopicName);
            
            _msg = new Image();
            _msg.Header = new Header();
            _msg.Header.Frame_id = FrameId;
            
            _msg.Width = (uint)width;
            _msg.Height = (uint)height;

            _msg.Encoding = "32FC1";
            _msg.Is_bigendian = 0;
            _msg.Step = _msg.Width * sizeof(float);
        }

        private void SetupIntrinsics()
        {
            var fy = height / (2.0f * Mathf.Tan(renderCamera.fieldOfView * Mathf.Deg2Rad * 0.5f));
            var fx = fy;

            var cx = width * 0.5f;
            var cy = height * 0.5f;

            _depthToPointShader.SetFloat("fx", fx);
            _depthToPointShader.SetFloat("fy", fy);
            _depthToPointShader.SetFloat("cx", cx);
            _depthToPointShader.SetFloat("cy", cy);
        }

        protected override void Publish()
        {
            var tx = Mathf.CeilToInt(width / 8.0f);
            var ty = Mathf.CeilToInt(height / 8.0f);
            _depthToPointShader.Dispatch(_kernel, tx, ty, 1);
            
            AsyncGPUReadback.Request(_depthBuffer, request =>
            {
                if (request.hasError)
                    return;
                
                var raw = request.GetData<float>();
                PublishDepthImage(raw);
            });

            // Point could
            if (_publishPointCloud)
            {
                AsyncGPUReadback.Request(_pointBuffer, OInPointsReady);
            }
        }

        private void PublishDepthImage(NativeArray<float> raw)
        {
            if (!Ros2System.IsOk) return;
            
            if (_bytes == null || _bytes.Length != raw.Length * sizeof(float))
                _bytes = new byte[raw.Length * sizeof(float)];

            var rawBytes = raw.Reinterpret<byte>(sizeof(float));
            rawBytes.CopyTo(_bytes);
            
            _msg.Data = _bytes;
            UpdateTimeStamp(ref _msg);
            publisher.Publish(_msg);
            
            // Publish Camera info
            base.Publish();
        }
        
        private void UpdateTimeStamp(ref Image image)
        {
            var clockMsg = new rosgraph_msgs.msg.Clock();
            Node.clock.UpdateClockMessage(ref clockMsg);

            image.UpdateHeaderTime(clockMsg.Clock_.Sec, clockMsg.Clock_.Nanosec);
        }

        private void OInPointsReady(AsyncGPUReadbackRequest req)
        {
            if (req.hasError)
                return;
            
            var raw = req.GetData<Vector3>();
            var rawBytes = raw.Reinterpret<byte>(sizeof(float) * 3);
            var msg = new PointCloud2();

            msg.Header = new Header();
            msg.Header.Frame_id = FrameId;
            // Time

            msg.Width = (uint)width;
            msg.Height = (uint)height;
            
            msg.Is_bigendian = false;
            msg.Is_dense = true;

            msg.Point_step = 12;
            msg.Row_step = msg.Point_step * msg.Width;
            
            msg.Fields = new PointField[3];
            msg.Fields[0] = CreateField("x", 0);
            msg.Fields[1] = CreateField("y", 4);
            msg.Fields[2] = CreateField("z", 8);
            msg.Data = rawBytes.ToArray();

            if (!Ros2System.IsOk) return;

            _pointCloudPublisher.Publish(msg);
        }
        

        private void OnDestroy()
        {
            if (_pointBuffer != null)
                _pointBuffer.Release();
        }
        
        private PointField CreateField(string fieldName, uint offset)
        {
            var f = new PointField();
            f.Name = fieldName;                 
            f.Offset = offset;
            f.Datatype = PointField.FLOAT32;
            f.Count = 1;
            return f;
        }
    }
}
