using System;
using UnityEngine;

[RequireComponent(typeof(PathSubscriber))]
public class PathVisualizer : MonoBehaviour
{
    [Header("Settings")]
    [SerializeField] private PathSubscriber _pathSubscriber;
    
    [Space]
    [Header("Line")]
    [SerializeField] private float _lineWidth = 0.1f;
    [SerializeField] private Color _lineColor = Color.green;

    [Header("Marker")]
    [SerializeField] private bool _drawGoalMarker;
    [SerializeField] private bool _drawStepMarker;
    [SerializeField] private int _stepMarkerCount = 5;
    
    [SerializeField] private Transform _goalMarkerMarkerPrefab;
    [SerializeField] private Transform _stepMarkerPrefab;
    
    private LineRenderer _lineRenderer;
    private Transform _goalMarkerTransform;
    private Transform[] _stepMarkers;
    private ROS2System _ros2System;
    private bool _shouldDraw;

    private void Awake()
    {
        if (_pathSubscriber == null)
            Debug.LogError($"{nameof(_pathSubscriber)} is not assigned");
        
        var obj = new GameObject("PathVisualizer");
        obj.transform.parent = transform;
        
        _lineRenderer = obj.AddComponent<LineRenderer>();
        
        SetUpLineRenderer();

        var debugLayer = LayerMask.NameToLayer("Debug");
        Debug.Log($"Setting Path visualizer : {debugLayer}");
        gameObject.layer = debugLayer;
        
        if (_drawGoalMarker)
        {
            _goalMarkerTransform = Instantiate(_goalMarkerMarkerPrefab);
            _goalMarkerTransform.gameObject.SetActive(false);
        }

        if (_drawStepMarker)
        {
            _stepMarkers = new Transform[_stepMarkerCount];
            for (var i = 0; i < _stepMarkerCount; i++)
            {
                var marker = Instantiate(_stepMarkerPrefab);
                
                marker.gameObject.SetActive(false);   
                _stepMarkers[i] = marker;
            }
        }
    }

    private void SetUpLineRenderer()
    {
        var pathShader = Shader.Find("Sprites/Default");
        var material = new Material(pathShader);
        material.SetColor("_BaseColor", _lineColor);
        
        _lineRenderer.useWorldSpace = true;
        _lineRenderer.positionCount = 0;
        _lineRenderer.numCornerVertices = 5;
        _lineRenderer.numCapVertices = 5;
        
        _lineRenderer.material = material;
        _lineRenderer.startWidth = _lineWidth;
        _lineRenderer.endWidth = _lineWidth;
        
        _lineRenderer.startColor = _lineColor;
        _lineRenderer.endColor = _lineColor;
        
        _lineRenderer.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
        _lineRenderer.receiveShadows = false;
        
        var gradient = new Gradient();
        gradient.SetKeys(
            new[] 
            { 
                new GradientColorKey(_lineColor, 0f), 
                new GradientColorKey(_lineColor, 1f) 
            },
            new[]
            {
                new GradientAlphaKey(1f, 0f), 
                new GradientAlphaKey(1f, 1f)
            }
        );

        _lineRenderer.colorGradient = gradient;

    }
    
    void UpdateLine(Vector3[] points)
    {
        _lineRenderer.positionCount = points.Length;

        for (int i = 0; i < points.Length; i++)
        {
            var pos = points[i];
            pos.y += 0.05f;

            _lineRenderer.SetPosition(i, pos);
        }
    }
    
    void UpdateStepMarkers(Vector3[] points)
    {
        var totalLength = 0f;

        for (var i = 1; i < points.Length; i++)
            totalLength += Vector3.Distance(points[i - 1], points[i]);

        var spacing = totalLength / _stepMarkerCount;

        var accumulated = 0f;
        var nextTarget = spacing;

        var markerIndex = 0;

        for (var i = 1; i < points.Length; i++)
        {
            if (markerIndex >= _stepMarkerCount)
                break;

            var segment = Vector3.Distance(points[i - 1], points[i]);
            accumulated += segment;

            if (accumulated >= nextTarget)
            {
                var pos = points[i];
                pos.y += 0.05f;

                var dir = (points[i] - points[i - 1]).normalized;

                var rot = Quaternion.LookRotation(dir, Vector3.up);

                var marker = _stepMarkers[markerIndex];
                marker.gameObject.SetActive(true);
                marker.SetPositionAndRotation(pos, rot);

                markerIndex++;
                nextTarget += spacing;
            }
        }
    }

    void UpdateGoalMarker(Vector3[] points)
    {
        var last = points.Length - 1;

        var pos = points[last];
        pos.y += 0.05f;

        var dir = (points[last] - points[last - 1]).normalized;
        var rot = Quaternion.LookRotation(dir);

        _goalMarkerTransform.SetPositionAndRotation(pos, rot);
    }
    
    private void Start()
    {
        _ros2System = ROS2System.Instance;
        _ros2System.OnPhaseChanged += Toggle;
    }

    private void Toggle(byte previousPhase, byte newPhase)
    {
        // Enter executing
        if (newPhase == SystemPhases.PHASE_NAV_EXECUTING)
        {
            _shouldDraw = true;
        }
        
        // Exit executing
        if (previousPhase == SystemPhases.PHASE_NAV_EXECUTING)
        {
            _shouldDraw = false;
            
            if (_drawStepMarker)
            {
                foreach (var stepMarker in _stepMarkers)
                {
                    stepMarker.gameObject.SetActive(false);
                }
            }

            if (_drawGoalMarker)
                _goalMarkerTransform.gameObject.SetActive(false);
        }
    }

    private void Update()
    {
        if (!_shouldDraw) return;
        if (!_pathSubscriber.dispatcher.TryDequeueLatest(out var frame)) return;
        
        UpdatePath(frame);
    }

    private void UpdatePath(PathFrame frame)
    {
        var points = frame.PathPoints;

        UpdateLine(points);

        if (_drawStepMarker)
            UpdateStepMarkers(points);

        if (_drawGoalMarker)
            UpdateGoalMarker(points);
    }
}
