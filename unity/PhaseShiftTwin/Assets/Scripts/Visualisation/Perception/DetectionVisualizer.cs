using System.Collections.Generic;
using Communication;
using geometry_msgs.msg;
using UnityEngine;
using Quaternion = UnityEngine.Quaternion;
using Vector3 = UnityEngine.Vector3;

[RequireComponent(typeof(DetectionObjectSubscriber))]
public class DetectionVisualizer : MonoBehaviour
{
    [Header("Settings")] 
    [SerializeField] private DetectionObjectSubscriber _objectSubscriber;
    
    [Space]
    public BoundingBox bboxPrefab;

    private Dictionary<int, BoundingBox> objects = new();
    private Dictionary<int, float> lastSeen = new();

    private int nextId = 0;

    private float matchDistance = 1.0f;
    private float timeout = 1.0f;
    private Material _material;

    private void Awake()
    {
        _material = new Material(Shader.Find("Sprites/Default"));
    }

    private void Update()
    {
        if (_objectSubscriber == null) return;
        if (!_objectSubscriber.dispatcher.TryDequeueLatest(out var frame)) return;
        
        OnObjects3D(frame);
    }

    // ==========================
    // ROS CALLBACK
    // ==========================
    public void OnObjects3D(DetectionObjectArrayFrame msg)
    {
        float now = Time.time;

        foreach (var obj in msg.DetectionObjects)
        {
            Vector3 pos = RosToUnity(obj.Pose.Position);

            int id = FindMatch(pos);

            if (id == -1)
            {
                id = CreateBBox(pos);
            }
            Debug.Log($"{obj.Class_Id} - {obj.Confidence} -  {obj.Pose.Position}");
            UpdateBBox(id, pos);
            lastSeen[id] = now;
        }

        Cleanup(now);
    }
    
    // ==========================
    // CREATE
    // ==========================
    int CreateBBox(Vector3 pos)
    {
        var bbox = Instantiate(bboxPrefab, pos, Quaternion.identity);
        bbox.Initialize(_material);
        bbox.transform.SetParent(transform);
        
        int id = nextId++;
        objects[id] = bbox;

        return id;
    }

    // ==========================
    // UPDATE
    // ==========================
    void UpdateBBox(int id, Vector3 pos)
    {
        var bbox = objects[id];

        bbox.UpdateBox(pos, Vector3.one * 0.5f);
    }

    // ==========================
    // MATCHING (임시 tracking)
    // ==========================
    int FindMatch(Vector3 pos)
    {
        foreach (var kv in objects)
        {
            float dist = Vector3.Distance(kv.Value.transform.position, pos);
            if (dist < matchDistance)
                return kv.Key;
        }
        return -1;
    }

    // ==========================
    // CLEANUP
    // ==========================
    void Cleanup(float now)
    {
        List<int> remove = new();

        foreach (var kv in lastSeen)
        {
            if (now - kv.Value > timeout)
            {
                Destroy(objects[kv.Key].gameObject);
                remove.Add(kv.Key);
            }
        }

        foreach (var id in remove)
        {
            objects.Remove(id);
            lastSeen.Remove(id);
        }
    }

    // ==========================
    // ROS → Unity 변환
    // ==========================
    Vector3 RosToUnity(Point point)
    {
        return new Vector3(
            -(float)point.Y,
            (float)point.Z,
            (float)point.X);
    }
}