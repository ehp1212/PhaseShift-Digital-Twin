using Communication;
using Communication.Thread;
using sensor_msgs.msg;

public struct PointCloud2Frame : IThreadFrame
{
    public int PointStep;
    public byte[] Data;
}

public class PointCloud2Subscriber : ROS2Subscriber<PointCloud2, PointCloud2Frame>
{
    protected override void SubscribeCallback(PointCloud2 msg)
    {
        var frame = new PointCloud2Frame()
        {
            PointStep = (int)msg.Point_step,
            Data = msg.Data
        };
        
        dispatcher.Push(frame);
    }
}