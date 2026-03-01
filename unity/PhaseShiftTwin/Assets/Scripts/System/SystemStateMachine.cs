using System.Collections.Generic;
using UnityEngine;

namespace System
{
    public static class SystemPhases
    {
        public const byte PHASE_INIT = 255; // Unity only state before receiving any state from ros2
        
        public const byte PHASE_BOOT = 0;
        public const byte PHASE_CHECK_MAP = 1;
        public const byte PHASE_CONNECTING = 2;
        public const byte PHASE_SLAM_ACTIVE = 3;
        public const byte PHASE_MAP_SAVED = 4;
        public const byte PHASE_NAV_READY = 5;
        public const byte PHASE_NAVIGATING = 6;
        public const byte PHASE_ERROR = 7;
    }
    
    public class SystemStateMachine
    {
        private Dictionary<byte, ISystemState> states = new();
        private ISystemState currentState;
        
        public byte Current => currentState?.Phase ?? SystemPhases.PHASE_INIT;

        public SystemStateMachine(ROS2System ros2System)
        {
            // Register states
            states[SystemPhases.PHASE_INIT] = new InitState(ros2System);
            states[SystemPhases.PHASE_BOOT] = new BootState(ros2System);
            states[SystemPhases.PHASE_CHECK_MAP] = new CheckMapState(ros2System);
            states[SystemPhases.PHASE_CONNECTING] = new ConnectingState(ros2System);
            states[SystemPhases.PHASE_SLAM_ACTIVE] = new SlamActiveState(ros2System);
            states[SystemPhases.PHASE_MAP_SAVED] = new MapSavedState(ros2System);
            states[SystemPhases.PHASE_NAV_READY] = new NavReadyState(ros2System);
            states[SystemPhases.PHASE_NAVIGATING] = new NavigatingState(ros2System);
            states[SystemPhases.PHASE_ERROR] = new ErrorState(ros2System);
            
            currentState = states[SystemPhases.PHASE_INIT];
        }

        public void ApplyPhase(byte newPhase, bool force = false)
        {
            if (!states.ContainsKey(newPhase))
            {
                Debug.LogError($"[FSM] Unknown phase: {newPhase}");
                return;
            }

            if (!force && currentState != null && currentState.Phase == newPhase)
                return; // same state → ignore

            currentState?.Exit();
            currentState = states[newPhase];
            currentState.Enter();
        }

        public void OnUpdate()
        {
            currentState?.Tick();
        }
    }
}