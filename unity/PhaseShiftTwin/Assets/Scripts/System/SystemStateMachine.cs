using System.Collections.Generic;
using UnityEngine;

namespace System
{
    public static class SystemPhases
    {
        public const byte PHASE_INIT = 255; // Unity only state before receiving any state from ros2
        
        public const byte PHASE_BOOT = 0;
        public const byte PHASE_CONNECTING = 1;
        public const byte PHASE_SLAM_ACTIVE = 2;
        public const byte PHASE_MAP_SAVED = 3;
        public const byte PHASE_NAV_READY = 4;
        public const byte PHASE_NAVIGATING = 5;
        public const byte PHASE_ERROR = 6;
    }
    
    public class SystemStateMachine
    {
        private Dictionary<byte, ISystemState> states = new();
        private byte _currentPhase = SystemPhases.PHASE_INIT;
        private ISystemState currentState;
        public byte Current => _currentPhase;

        public SystemStateMachine()
        {
            states[SystemPhases.PHASE_INIT] = new InitState();
            states[SystemPhases.PHASE_BOOT] = new BootState();
            states[SystemPhases.PHASE_CONNECTING] = new ConnectingState();
            states[SystemPhases.PHASE_SLAM_ACTIVE] = new SlamActiveState();
            states[SystemPhases.PHASE_MAP_SAVED] = new MapSavedState();
            states[SystemPhases.PHASE_NAV_READY] = new NavReadyState();
            states[SystemPhases.PHASE_NAVIGATING] = new NavigatingState();
            states[SystemPhases.PHASE_ERROR] = new ErrorState();
        }

        public void ApplyPhase(byte newPhase)
        {
            if (!states.ContainsKey(newPhase))
            {
                Debug.LogError($"[FSM] Unknown phase: {newPhase}");
                return;
            }

            if (currentState != null && currentState.Phase == newPhase)
                return; // same state → ignore

            currentState?.Exit();
            _currentPhase = newPhase;
            currentState = states[newPhase];
            currentState.Enter();
        }

        public void OnUpdate()
        {
            currentState?.Tick();
        }
    }
}