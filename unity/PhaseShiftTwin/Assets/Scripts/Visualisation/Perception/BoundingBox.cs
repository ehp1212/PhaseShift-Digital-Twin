using UnityEngine;

public class BoundingBox : MonoBehaviour
{
    private LineRenderer[] _lineRenderers = new LineRenderer[12];
    
    private static readonly int EDGE_COUNT = 12;

    private void Awake()
    {
        var layer = LayerMask.NameToLayer("Debug");
        gameObject.layer = layer;
        
        for (int i = 0; i < EDGE_COUNT; i++)
        {
            var obj = new GameObject($"edge_{i}");
            obj.transform.SetParent(transform);
            
            var lr = obj.gameObject.AddComponent<LineRenderer>();
            lr.startWidth = 0.02f;
            lr.endWidth = 0.02f;
            lr.positionCount = 2;
            lr.useWorldSpace = true;
            
            _lineRenderers[i] = lr;
        }
    }

    public void Initialize(Material sharedMaterial)
    {
        for (int i = 0; i < EDGE_COUNT; i++)
        {
            _lineRenderers[i].sharedMaterial = sharedMaterial;
        }
    }

    public void UpdateBox(Vector3 center, Vector3 size)
    {
        var half = size * 0.5f;
        var p = new Vector3[8];
        
        p[0] = center + new Vector3(-half.x, -half.y, -half.z);
        p[1] = center + new Vector3( half.x, -half.y, -half.z);
        p[2] = center + new Vector3( half.x, -half.y,  half.z);
        p[3] = center + new Vector3(-half.x, -half.y,  half.z);

        p[4] = center + new Vector3(-half.x,  half.y, -half.z);
        p[5] = center + new Vector3( half.x,  half.y, -half.z);
        p[6] = center + new Vector3( half.x,  half.y,  half.z);
        p[7] = center + new Vector3(-half.x,  half.y,  half.z);
        
        // bottom
        SetLine(0, p[0], p[1]);
        SetLine(1, p[1], p[2]);
        SetLine(2, p[2], p[3]);
        SetLine(3, p[3], p[0]);

        // top
        SetLine(4, p[4], p[5]);
        SetLine(5, p[5], p[6]);
        SetLine(6, p[6], p[7]);
        SetLine(7, p[7], p[4]);

        // vertical
        SetLine(8, p[0], p[4]);
        SetLine(9, p[1], p[5]);
        SetLine(10, p[2], p[6]);
        SetLine(11, p[3], p[7]);
    }
    
    private void SetLine(int index, Vector3 a, Vector3 b)
    {
        var lineRenderer = _lineRenderers[index];
        lineRenderer.SetPosition(0, a);
        lineRenderer.SetPosition(1, b);
    }
}