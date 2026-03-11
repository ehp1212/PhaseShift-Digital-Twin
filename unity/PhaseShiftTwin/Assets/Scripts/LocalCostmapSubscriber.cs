using Communication;
using Communication.Thread;
using nav_msgs.msg;
using phaseshift_interfaces.msg;
using TreeEditor;
using UnityEngine;

public struct CostmapFrame : IThreadFrame
{
    public int Width;
    public int Height;
    
    public float Resolution;

    public Vector2 Origin;    
    public float OriginYaw;

    public byte[] Data;
}

public class LocalCostmapSubscriber : ROS2Subscriber<CostmapGrid, CostmapFrame>
{
    protected override void SubscribeCallback(CostmapGrid msg)
    {
        var frame = new CostmapFrame
        {
            Width = (int)msg.Width,
            Height = (int)msg.Height,
            Resolution = msg.Resolution,

            Origin = new Vector2(
                msg.Origin_x,
                msg.Origin_y
            ),

            OriginYaw = msg.Origin_yaw,

            Data = (byte[])msg.Data.Clone()
        };
        
        dispatcher.Push(frame);
    }
}
