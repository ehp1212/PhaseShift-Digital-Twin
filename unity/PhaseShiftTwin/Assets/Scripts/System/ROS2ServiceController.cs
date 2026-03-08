using System.Service;
using System.Threading.Tasks;
using phaseshift_interfaces.srv;
using ROS2;
using UnityEngine;

namespace System
{
    public class ROS2ServiceController
    {
        private ROS2System _ros2System;
        private ROS2Node _ros2Node;
        
        private SaveMapClient _saveMapClient;
        private SetGoalClient _setGoalClient;
        private NavigateClient _navigateClient;

        public ROS2ServiceController(ROS2System ros2System)
        {
            _ros2System = ros2System;
        }

        public void Initialize()
        {
            _ros2Node = _ros2System.CreateNode("unity_service_controller");
            _saveMapClient = new SaveMapClient(_ros2Node, "/system/save_map");
            _setGoalClient = new SetGoalClient(_ros2Node, "/system/set_goal");
            _navigateClient = new NavigateClient(_ros2Node, "/system/navigate");
        }

        public async Task SaveMap(string mapName)
        {
            try
            {
                var request = new SaveMap_Request()
                {
                    Map_name = mapName
                };

                var response = await _saveMapClient.CallAsync(request);
                Debug.Log($"[Response : {response.Success}]  {response.Message}]");
            }
            catch (Exception e)
            {
                Debug.LogError($"SaveMap failed: {e.Message}");
                throw;
            }
        }
    }
}