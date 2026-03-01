using System.Utility;

namespace System
{
    public interface ISystemState
    {
        byte Phase { get; }

        void Enter();
        void Exit();
        void Tick();  
    }

    public abstract class PhaseSystemState : ISystemState
    {
        protected readonly ROS2System ros2System;
        public byte Phase { get; set; }
        public abstract void Enter();
        public abstract void Exit();
        public abstract void Tick();

        public PhaseSystemState(ROS2System ros2System)
        {
            this.ros2System = ros2System;
        }
    }

    public class InitState : PhaseSystemState
    {
        public InitState(ROS2System ros2System) : base(ros2System)
        {
            Phase = SystemPhases.PHASE_INIT;
        }

        public override void Enter()
        {
            StateLoggerUtility.LogState<InitState>();
            ros2System.LogScreenUI("Waiting for a message from PhaseShift package...");
        }

        public override void Exit()
        {
            ros2System.CloseScreenUI();
        }

        public override void Tick()
        {
        }
    }
    
    public class BootState : PhaseSystemState
    {
        public BootState(ROS2System ros2System) : base(ros2System)
        {
            Phase = SystemPhases.PHASE_BOOT;
        }

        public override void Enter()
        {
            StateLoggerUtility.LogState<BootState>();
            ros2System.LogScreenUI("Waiting for the boot process of PhaseShift package to complete...");
        }

        public override void Exit()
        {
            ros2System.CloseScreenUI();
        }

        public override void Tick()
        {
        }
    }

    public class CheckMapState : PhaseSystemState
    {
        public CheckMapState(ROS2System ros2System) : base(ros2System)
        {
            Phase = SystemPhases.PHASE_CHECK_MAP;
        }

        public override void Enter()
        {
            StateLoggerUtility.LogState<CheckMapState>();
        }

        public override void Exit()
        {
        }

        public override void Tick()
        {
        }
    }
    
    public class ConnectingState : PhaseSystemState
    {
        public ConnectingState(ROS2System ros2System) : base(ros2System)
        {
            Phase = SystemPhases.PHASE_CONNECTING;
        }

        public override void Enter()
        {
            StateLoggerUtility.LogState<ConnectingState>();
            
            // Start 2d scanner to provide /scan topic
            ros2System.ToggleScan(true);
        }

        public override void Exit()
        {
        }

        public override void Tick()
        {
        }
    }
    
    public class SlamActiveState : PhaseSystemState
    {
        public SlamActiveState(ROS2System ros2System) : base(ros2System)
        {
            Phase = SystemPhases.PHASE_SLAM_ACTIVE;
        }

        public override void Enter()
        {
        }

        public override void Exit()
        {
            ros2System.ToggleScan(false);
        }

        public override void Tick()
        {
        }
    }
    
    public class MapSavedState : PhaseSystemState
    {
        public MapSavedState(ROS2System ros2System) : base(ros2System)
        {
            Phase = SystemPhases.PHASE_MAP_SAVED;
        }

        public override void Enter()
        {
        }

        public override void Exit()
        {
        }

        public override void Tick()
        {
        }
    }
    
    public class NavReadyState : PhaseSystemState
    {
        public NavReadyState(ROS2System ros2System) : base(ros2System)
        {
            Phase = SystemPhases.PHASE_NAV_READY;
        }

        public override void Enter()
        {
        }

        public override void Exit()
        {
        }

        public override void Tick()
        {
        }
    }
    
    public class NavigatingState : PhaseSystemState
    {
        public NavigatingState(ROS2System ros2System) : base(ros2System)
        {
            Phase = SystemPhases.PHASE_NAVIGATING;
        }

        public override void Enter()
        {
        }

        public override void Exit()
        {
        }

        public override void Tick()
        {
        }
    }
    
    public class ErrorState : PhaseSystemState
    {
        public ErrorState(ROS2System ros2System) : base(ros2System)
        {
            Phase = SystemPhases.PHASE_ERROR;
        }

        public override void Enter()
        {
        }

        public override void Exit()
        {
        }

        public override void Tick()
        {
        }
    }
}