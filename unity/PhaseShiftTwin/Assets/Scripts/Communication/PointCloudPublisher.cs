using System;
using UnityEngine;
using UnitySensors.Attribute;
using UnitySensors.DataType.Sensor.PointCloud;
using UnitySensors.Interface.Sensor;
using UnitySensors.Sensor;
using UnitySensors.Sensor.LiDAR;
using sensor_msgs.msg;
using std_msgs.msg;

namespace Communication
{
    [RequireComponent(typeof(RaycastLiDARSensor))]
    public class PointCloudPublisher : Publisher<PointCloud2>
    {
        [Interface(typeof(IPointCloudInterface<PointXYZI>))]
        private UnitySensor _source;
        
        private IPointCloudInterface<PointXYZI> _sourceInterface;

        protected override void OnInitialize()
        {
            base.OnInitialize();
            
            _source.onSensorUpdated += Publish;
        }

        protected override void Start()
        {
            base.Start();
            
            _source = GetComponent<UnitySensor>();
            _sourceInterface = _source as IPointCloudInterface<PointXYZI>;
            if (_sourceInterface == null)
            {
                Debug.LogError($"Cannot get source interface");
                return;
            }
        }

        protected override void Publish()
        {
            var cloudPoints = _sourceInterface.pointCloud.points;
            
            // test timestamp
            int sec = 0;
            uint nanosec = 0;

            var msg = BuildPointCloud2FromXYZI(
                cloudPoints,
                FrameId,   // frame_id
                sec,
                nanosec
            );

            publisher.Publish(msg);
        }
        
        private PointCloud2 BuildPointCloud2FromXYZI(
            Unity.Collections.NativeArray<PointXYZI> points,
            string frameId,
            int sec,
            uint nanosec)
        {
            int count = points.Length;

            var msg = new PointCloud2();

            // Header
            msg.Header = new Header();
            msg.Header.Frame_id = frameId;
            msg.Header.Stamp.Sec = sec;
            msg.Header.Stamp.Nanosec = nanosec;

            // Organized cloud가 아니면 보통 height=1, width=N
            msg.Height = 1;
            msg.Width  = (uint)count;

            msg.Is_bigendian = false;
            msg.Is_dense = true;

            // x,y,z,intensity float32 4개 = 16 bytes
            msg.Point_step = 16;
            msg.Row_step   = msg.Point_step * msg.Width;

            // Fields 정의 (offset 중요)
            msg.Fields = new PointField[4];
            msg.Fields[0] = CreateField("x", 0);
            msg.Fields[1] = CreateField("y", 4);
            msg.Fields[2] = CreateField("z", 8);
            msg.Fields[3] = CreateField("intensity", 12);

            // Data 크기: Height * Row_step
            int dataSize = checked((int)(msg.Height * msg.Row_step));
            msg.Data = new byte[dataSize];

            for (int i = 0; i < count; i++)
            {
                var p = points[i];

                // Unity → ROS 좌표 변환 (권장)
                float ros_x = p.position.z;
                float ros_y = -p.position.x;
                float ros_z = p.position.y;

                int offset = i * 16;
                WriteFloatLE(msg.Data, offset + 0,  ros_x);
                WriteFloatLE(msg.Data, offset + 4,  ros_y);
                WriteFloatLE(msg.Data, offset + 8,  ros_z);
                WriteFloatLE(msg.Data, offset + 12, p.intensity);
            }

            return msg;
        }

        private PointField CreateField(string name, uint offset)
        {
            var f = new PointField();
            f.Name = name;                 // PointField도 DLL마다 Name/name 차이 있을 수 있음 (아래 주의 참고)
            f.Offset = offset;
            f.Datatype = PointField.FLOAT32;
            f.Count = 1;
            return f;
        }

        private void WriteFloatLE(byte[] buffer, int offset, float value)
        {
            // 대부분 리틀엔디안. (Is_bigendian=false)
            var bytes = BitConverter.GetBytes(value);
            Buffer.BlockCopy(bytes, 0, buffer, offset, 4);
        }
    }
}
