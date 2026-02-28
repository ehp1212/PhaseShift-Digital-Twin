using UnityEngine;

namespace System.Utility
{
    public static class StateLoggerUtility
    {
        public static void LogState<T>() where T : PhaseSystemState
        {
            Debug.Log($"<color=green>[FSM]</color> Start apply phase - {typeof(T)}");
        }
    }
}