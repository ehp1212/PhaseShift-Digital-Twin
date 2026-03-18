using System;
using UnityEngine;

namespace Communication.Camera
{
    [RequireComponent(typeof(UnityEngine.Camera))]
    public class HDRPRGBCompressedCamera : MonoBehaviour
    {
        [SerializeField] private Vector2Int _resolution =  new Vector2Int(640, 480);
        
        private UnityEngine.Camera _camera;
        private RenderTexture _rt = null;

        public RenderTexture RenderTexture => _rt;
        public Vector2Int Resolution => _resolution;

        private void Awake()
        {
            _rt = new RenderTexture(_resolution.x, _resolution.y, 0, RenderTextureFormat.ARGB32);
        }

        private void Start()
        {
            _camera = GetComponent<UnityEngine.Camera>();
            _camera.targetTexture = _rt;
        }
    }
}
