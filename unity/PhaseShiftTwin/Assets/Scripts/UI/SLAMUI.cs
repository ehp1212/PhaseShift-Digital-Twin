using System;
using UnityEngine;
using UnityEngine.UI;

namespace UI
{
    public class SLAMUI : MonoBehaviour
    {
        [SerializeField] private Button _saveMapButton;
        [SerializeField] private Toggle _visualizePT;

        private ROS2System _ros2System;
        private void Start()
        {
            _ros2System = ROS2System.Instance;
            _saveMapButton.onClick.AddListener(SaveMap);
        }

        private async void SaveMap()
        {
            const string mapName = "default";
            await _ros2System.ROS2ServiceController.SaveMap(mapName);
        }

        public void Toggle(bool toggle)
        {
            gameObject.SetActive(toggle);
        }
    }
}
