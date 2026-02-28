using System.Utility;
using UnityEngine;

namespace System
{
    public interface ISystemState
    {
        byte Phase { get; }

        void Enter();
        void Exit();
        void Tick();   // optional (UI animation 등)
    }

    public abstract class PhaseSystemState : ISystemState
    {
        public byte Phase { get; }
        public abstract void Enter();
        public abstract void Exit();
        public abstract void Tick();
    }

    public class InitState : PhaseSystemState
    {
        public override void Enter()
        {
            StateLoggerUtility.LogState<InitState>();
            Debug.Log($"Waiting for a message from PhaseShift package...");
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
        public override void Enter()
        {
            StateLoggerUtility.LogState<BootState>();
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
        public override void Enter()
        {
            StateLoggerUtility.LogState<ConnectingState>();
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
    
    public class MapSavedState : PhaseSystemState
    {
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