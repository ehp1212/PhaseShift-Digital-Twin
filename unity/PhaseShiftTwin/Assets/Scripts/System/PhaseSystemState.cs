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

    public class SystemInitState : PhaseSystemState
    {
        public SystemInitState(ROS2System ros2System) : base(ros2System)
        {
            Phase = SystemPhases.PHASE_SYSTEM_INITIALIZING;
        }

        public override void Enter()
        {
            StateLoggerUtility.LogState<SystemInitState>();
            ros2System.ToggleNav(false);
        }

        public override void Exit()
        {
            ros2System.CloseScreenUI();
        }

        public override void Tick()
        {
        }
    }
    
    public class SLAMPrepState : PhaseSystemState
    {
        public SLAMPrepState(ROS2System ros2System) : base(ros2System)
        {
            Phase = SystemPhases.PHASE_SLAM_PREPARING;
        }

        public override void Enter()
        {
            StateLoggerUtility.LogState<SLAMPrepState>();
            
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
    
    public class SLAMActiveState : PhaseSystemState
    {
        public SLAMActiveState(ROS2System ros2System) : base(ros2System)
        {
            Phase = SystemPhases.PHASE_SLAM_ACTIVE;
        }

        public override void Enter()
        {
            StateLoggerUtility.LogState<SLAMActiveState>();
            
            // Turn on manual driving
            ros2System.StartManualDriving(true);
            ros2System.EnterSlamPhase(true);
        }

        public override void Exit()
        {
            ros2System.ToggleScan(false);
            ros2System.StartManualDriving(false);
            ros2System.EnterSlamPhase(false);
        }

        public override void Tick()
        {
        }
    }
    
    public class SLAMMapSaving : PhaseSystemState
    {
        public SLAMMapSaving(ROS2System ros2System) : base(ros2System)
        {
            Phase = SystemPhases.PHASE_MAP_SAVING;
        }

        public override void Enter()
        {
            StateLoggerUtility.LogState<SLAMMapSaving>();
            ros2System.LogScreenUI("Trying to save map...");
        }

        public override void Exit()
        {
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
            ros2System.LogScreenUI("Completed to save map successfully");
        }

        public override void Exit()
        {
            ros2System.CloseScreenUI();
        }

        public override void Tick()
        {
        }
    }
    
    public class NAVPrepState : PhaseSystemState
    {
        public NAVPrepState(ROS2System ros2System) : base(ros2System)
        {
            Phase = SystemPhases.PHASE_NAV_PREPARING;
        }

        public override void Enter()
        {
            StateLoggerUtility.LogState<NAVPrepState>();
            ros2System.LogScreenUI("Trying to load NAV2 in ROS2 side...");
            
            // TODO: when to turn off
            ros2System.ToggleScan(true);
        }

        public override void Exit()
        {
            ros2System.CloseScreenUI();
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
            StateLoggerUtility.LogState<NavReadyState>();
            
            // Local costmap sub / visual
            // Detect sub / visual
            ros2System.ToggleNav(true);
        }

        public override void Exit()
        {
            // Clear in init phase
            // ros2System.ToggleNav(false);
        }

        public override void Tick()
        {
        }
    }
    
    public class NAVExecuteState : PhaseSystemState
    {
        public NAVExecuteState(ROS2System ros2System) : base(ros2System)
        {
            Phase = SystemPhases.PHASE_NAV_EXECUTING;
        }

        public override void Enter()
        {
            // Nav Feedback 
            // Global path
            // Local path
            ros2System.ToggleNavExec(true);
        }

        public override void Exit()
        {
            ros2System.ToggleNavExec(false);
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