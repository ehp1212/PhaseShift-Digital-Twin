using Communication.Thread;
using Sensor.Visualizer;
using UnityEngine;

[RequireComponent(typeof(LocalCostmapSubscriber))]
public class LocalCostmapVisualizer : HDRPPointCloudVisualizerProcedural
{
    private LocalCostmapSubscriber _localCostmapSubscriber;
    private PointXYZRGB[] points;

    void Start()
    {
        _localCostmapSubscriber = GetComponent<LocalCostmapSubscriber>();
    }

    protected override void Update()
    {
        base.Update();
        if (!_localCostmapSubscriber.dispatcher.TryDequeueLatest(out var frame))
            return;

        RenderCostmap(frame);
    }
    
    private void RenderCostmap(CostmapFrame frame)
    {
        var width = frame.Width;
        var height = frame.Height;
        var res = frame.Resolution;

        var maxCount = width * height;
        points = new PointXYZRGB[maxCount];

        var index = 0;
        for (var row = 0; row < height; row++)
        {
            for (var col = 0; col < width; col++)
            {
                var gridIndex = row * width + col;
                
                var occ = frame.Data[gridIndex];

                if (occ < 5) continue;
                
                var rosX = frame.Origin.x + col * res;
                var rosY = frame.Origin.y + row * res;

                var pos = new Vector3(
                    -rosY,
                    0.1f,
                    rosX
                );

                points[index] = new PointXYZRGB
                {
                    Position = pos,
                    ColorRGB = CostToColor(occ)
                };
                
                index++;
            }
        }
        
        UpdatePointCloud(points, points.Length);
    }
    
    private uint CostToColor(byte occ)
    {
        if (occ == 0)
            return 0;

        var t = occ / 100f;

        Color color;

        if (t < 0.5f)
            color = Color.Lerp(Color.blue, Color.yellow, t * 2f);
        else
            color = Color.Lerp(Color.yellow, Color.red, (t - 0.5f) * 2f);

        return PackColor(
            (byte)(color.r * 255),
            (byte)(color.g * 255),
            (byte)(color.b * 255)
        );
    }
    
    private static uint PackColor(byte r, byte g, byte b)
    {
        return (uint)(r << 16 | g << 8 | b);
    }
}
