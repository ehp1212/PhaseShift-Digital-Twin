using UnityEngine;

namespace UI
{
    [CreateAssetMenu(menuName = "UI Context")]
    public class UIContext : ScriptableObject
    {
        [field: SerializeField] public ScreenUI ScreenUI { get; private set; }
    }
}
