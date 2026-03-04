using Communication;
using Communication.Thread;
using Sensor.Visualizer;
using UnityEngine;

[RequireComponent(typeof(OccupancyGridRos2Subscriber))]
public class OccupancyGridVisualizer : HDRPPointCloudVisualizerProcedural
{
    private OccupancyGridRos2Subscriber _occupancyGridRos2Subscriber;
    private PointXYZRGB[] _points;
    
    protected override void Awake()
    {
        base.Awake();
        _occupancyGridRos2Subscriber = GetComponent<OccupancyGridRos2Subscriber>();
    }

    protected override void Update()
    {
        base.Update();
        
        // Unity main thread
        if (_occupancyGridRos2Subscriber.dispatcher.TryDequeueLatest(out var frame))
        {
            Visualize(frame);
        }
    }

    private void Visualize(OccupancyGridFrame frame)
    {
        var width = frame.Width;
        var height = frame.Height;
        var total = width * height;

        // allocate buffer once
        if (_points == null || _points.Length < total)
            _points = new PointXYZRGB[total];

        var index = 0;
        for (var row = 0; row < height; row++)
        {
            for (var col = 0; col < width; col++)
            {
                var gridIndex = row * width + col;
                var occ = frame.Data[gridIndex];
                if (occ < 50) continue;
                if (!TryConvertOccupancyToColor(occ, out uint color))
                    continue;
                
                float rosX = frame.OriginRos.x + (col * 2) * frame.Resolution;
                float rosY = frame.OriginRos.z + (row * 2) * frame.Resolution;
                
                // ROS → Unity
                Vector3 pos = new Vector3(
                    -rosY,
                    0.05f,
                    rosX
                );

                _points[index++] = new PointXYZRGB
                {
                    Position = pos,
                    ColorRGB = color
                };
                
                const int layers = 10;
                const float heightStep = 1f;
                for (int h = 0; h < layers; h++)
                {
                    _points[index++] = new PointXYZRGB
                    {
                        Position = new Vector3(
                            pos.x,
                            h * heightStep,
                            pos.z
                        ),
                        ColorRGB = color
                    };
                }
            }
        }

        UpdatePointCloud(_points, index);
    }
    
    private bool TryConvertOccupancyToColor(sbyte occ, out uint color)
    {
        // Free
        if (occ < 50)
        {
            color = PackColor(0, 255, 255);
            return true; 
        }
        
        // Occupied
        if (occ > 50)
        {
            color = PackColor(0, 0, 0); // black
            return true;
        }

        // fallback
        color = 0;
        return false;
    }
    
    uint PackColor(byte r, byte g, byte b, byte a = 255)
    {
        return (uint)(r | (g << 8) | (b << 16) | (a << 24));
    }
}
