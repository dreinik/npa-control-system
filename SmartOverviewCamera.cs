using UnityEngine;

[RequireComponent(typeof(Camera))]
public class OverviewCamera : MonoBehaviour
{
    [Header("Camera Settings")]
    [SerializeField] private float fixedHeight = 50f;
    [SerializeField] private float orthoSize = 30f;
    [SerializeField] private Vector3 fixedRotation = new Vector3(90f, 0f, 0f);

    private Camera cam;

    void Awake()
    {
        cam = GetComponent<Camera>();
        ResetCameraPosition();
    }

    public void ResetCameraPosition()
    {
        if (!gameObject.activeSelf) return;

        transform.position = new Vector3(0, fixedHeight, 0);
        transform.rotation = Quaternion.Euler(fixedRotation);

        if (TryGetComponent<Camera>(out var cam) && cam.orthographic)
        {
            cam.orthographicSize = orthoSize;
        }
    }
}