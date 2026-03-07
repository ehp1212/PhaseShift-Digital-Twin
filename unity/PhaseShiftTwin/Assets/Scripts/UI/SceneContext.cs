using Sensor.Lidar._2D;
using UnityEngine;

namespace UI
{
    [CreateAssetMenu(menuName = "UI Context")]
    public class SceneContext : ScriptableObject
    {
        [field: Header("UI")]
        [field: SerializeField] public ScreenUI ScreenUI { get; private set; }
        [field: SerializeField] public SLAMUI SlamUI { get; private set; }
        
        [field: Header("Sensor")]
        [field: SerializeField] public ScanRaycastSensor ScanRaycastSensor { get; private set; }
    }
}
