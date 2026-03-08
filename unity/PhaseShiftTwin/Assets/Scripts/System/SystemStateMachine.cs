using System.Collections.Generic;
using UnityEngine;

namespace System
{
    public static class SystemPhases
    {
        public const byte PHASE_INIT = 255; // Unity only state before receiving any state from ros2
        
        public const byte PHASE_BOOT = 0;
        public const byte PHASE_SYSTEM_INITIALIZING = 1;
        public const byte PHASE_SLAM_PREPARING = 2;
        public const byte PHASE_SLAM_ACTIVE = 3;
        public const byte PHASE_MAP_SAVING = 4;
        public const byte PHASE_MAP_SAVED = 5;
        public const byte PHASE_NAV_PREPARING = 6;
        public const byte PHASE_NAV_READY = 7;
        public const byte PHASE_NAV_EXECUTING = 8;
        public const byte PHASE_ERROR = 9;
    }
    
    public class SystemStateMachine
    {
        private Dictionary<byte, ISystemState> states = new();
        private ISystemState currentState;
        private readonly ROS2System _ros2System;

        public byte Current => currentState?.Phase ?? SystemPhases.PHASE_INIT;

        public SystemStateMachine(ROS2System ros2System)
        {
            _ros2System = ros2System;
            
            // Register states
            states[SystemPhases.PHASE_INIT] = new InitState(ros2System);
            
            states[SystemPhases.PHASE_BOOT] = new BootState(ros2System);
            states[SystemPhases.PHASE_SYSTEM_INITIALIZING] = new SystemInitState(ros2System);
            
            states[SystemPhases.PHASE_SLAM_PREPARING] = new SLAMPrepState(ros2System);
            states[SystemPhases.PHASE_SLAM_ACTIVE] = new SLAMActiveState(ros2System);
            states[SystemPhases.PHASE_MAP_SAVING] = new SLAMMapSaving(ros2System);
            states[SystemPhases.PHASE_MAP_SAVED] = new MapSavedState(ros2System);
            
            states[SystemPhases.PHASE_NAV_PREPARING] = new NAVPrepState(ros2System);
            states[SystemPhases.PHASE_NAV_READY] = new NavReadyState(ros2System);
            states[SystemPhases.PHASE_NAV_EXECUTING] = new NAVExecuteState(ros2System);
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

            if (currentState != null)
            {
                var previous = currentState.Phase;
                _ros2System.OnPhaseChanged?.Invoke(previous, newPhase);
            }

            currentState = states[newPhase];
            currentState.Enter();
        }

        public void OnUpdate()
        {
            currentState?.Tick();
        }
    }
}