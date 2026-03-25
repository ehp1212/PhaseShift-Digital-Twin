using System;
using Communication;
using Communication.Thread;
using phaseshift_interfaces.msg;
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

public class LocalCostmapSubscriber : ROS2Subscriber<CostmapGrid, CostmapFrame>, IROS2Interface
{
    public bool Active { get; set; }
    public void Toggle(bool active)
    {
        Active = active;
    }
    
    protected override void SubscribeCallback(CostmapGrid msg)
    {
        if (!Active) return;
        
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
