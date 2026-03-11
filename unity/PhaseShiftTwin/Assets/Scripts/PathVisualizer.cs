using UnityEngine;

[RequireComponent(typeof(PathSubscriber))]
public class PathVisualizer : MonoBehaviour
{
    private PathSubscriber _pathSubscriber;
    private LineRenderer _lineRenderer;

    private void Awake()
    {
        _pathSubscriber = GetComponent<PathSubscriber>();
        
        var obj = new GameObject("PathVisualizer");
        obj.transform.parent = transform;
        
        _lineRenderer = obj.AddComponent<LineRenderer>();
        _lineRenderer.positionCount = 0;
        _lineRenderer.startWidth = 0.2f;
        _lineRenderer.endWidth = 0.2f;
    }

    private void Update()
    {
        if (!_pathSubscriber.dispatcher.TryDequeueLatest(out var frame)) return;
        
        UpdatePath(frame);
    }

    private void UpdatePath(PathFrame frame)
    {
        _lineRenderer.positionCount = frame.PathPoints.Length;
        for (var i = 0; i < frame.PathPoints.Length; i++)
        {
            var point = frame.PathPoints[i];
            _lineRenderer.SetPosition(i, point);
        }
    }
}
