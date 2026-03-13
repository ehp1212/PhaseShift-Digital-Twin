using System;
using System.Threading.Tasks;
using UnityEngine;

public enum InputMode
{
    SLAM,
    NAV2
}
/*
데이터	                ROS Source
Goal Marker	            SetGoal request
Robot Pose	            TF
Global Costmap	        /global_costmap/costmap
Local Costmap	        /local_costmap/costmap
Global Path Preview	    /planner_server
*/

public class InputHandler : MonoBehaviour
{
    [SerializeField]
    private Camera mainCamera;

    [SerializeField]
    private LayerMask floorLayer;

    private ROS2System _ros2System;

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        mainCamera ??= Camera.main;
        _ros2System = ROS2System.Instance;
    }

    // Update is called once per frame
    void Update()
    {
        if (!Input.GetMouseButtonDown(0)) return;

        _ = TryDetectFloorPoint();
    }

    private async Task TryDetectFloorPoint()
    {
        var ray = mainCamera.ScreenPointToRay(Input.mousePosition);
        if (!Physics.Raycast(ray, out var hit, Mathf.Infinity, floorLayer))
            return;
        
        var worldPoint = hit.point;
        
        await SendGoal(worldPoint);
    }

    private async Task SendGoal(Vector3 worldPoint)
    {
        await _ros2System.ROS2ServiceController.SendGoal(worldPoint);
    }
}
