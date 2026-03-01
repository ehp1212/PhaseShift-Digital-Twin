using Sensor.Lidar._2D;
using UnityEngine;

namespace UI
{
    [CreateAssetMenu(menuName = "UI Context")]
    public class SceneContext : ScriptableObject
    {
        [Header("UI")]
        [field: SerializeField] public ScreenUI ScreenUI { get; private set; }
        
        [Header("Sensor")]
        [field: SerializeField] public ScanRaycastSensor ScanRaycastSensor { get; private set; }
    }
}
