using System;
using UnityEngine;
using UnitySensors.Attribute;
using UnitySensors.DataType.Sensor.PointCloud;
using UnitySensors.Interface.Sensor;
using UnitySensors.Sensor;
using UnitySensors.Sensor.LiDAR;
using sensor_msgs.msg;
using std_msgs.msg;
using Unity.Collections;

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
            if (!Ros2System.IsOk) return;
            
            var cloudPoints = _sourceInterface.pointCloud.points;
            
            var msg = BuildPointCloud2FromXYZI(
                cloudPoints,
                FrameId  // frame_id
            );

            UpdateTimeStamp(ref msg);
            publisher.Publish(msg);
        }
        
        private PointCloud2 BuildPointCloud2FromXYZI(
            NativeArray<PointXYZI> points,
            string frameId)
        {
            var count = points.Length;
            
            var rosPoints = new NativeArray<PointXYZI>(count, Allocator.Temp);
            for (var i = 0; i < count; i++)
            {
                var p = points[i];

                rosPoints[i] = new PointXYZI
                {
                    position = new Vector3(
                        p.position.z,
                        -p.position.x,
                        p.position.y),
                    intensity = p.intensity
                };
            }
            
            var msg = new PointCloud2();

            // Header
            msg.Header = new Header();
            msg.Header.Frame_id = frameId;

            // Organized cloud가 아니면 보통 height=1, width=N
            msg.Height = 1;
            msg.Width  = (uint)count;

            msg.Is_bigendian = false;
            msg.Is_dense = true;

            // x,y,z,intensity
            msg.Point_step = 16;
            msg.Row_step   = msg.Point_step * msg.Width;

            // Fields 정의 (offset 중요)
            msg.Fields = new PointField[4];
            msg.Fields[0] = CreateField("x", 0);
            msg.Fields[1] = CreateField("y", 4);
            msg.Fields[2] = CreateField("z", 8);
            msg.Fields[3] = CreateField("intensity", 12);

            var rawBytes = rosPoints.Reinterpret<byte>(16);
            msg.Data = rawBytes.ToArray();
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
        
        private void UpdateTimeStamp(ref PointCloud2 msg)
        {
            var clockMsg = new rosgraph_msgs.msg.Clock();
            Node.clock.UpdateClockMessage(ref clockMsg);

            msg.UpdateHeaderTime(clockMsg.Clock_.Sec, clockMsg.Clock_.Nanosec);
        }
    }
}
