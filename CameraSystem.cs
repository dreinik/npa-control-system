using UnityEngine;

public class CameraSystem : MonoBehaviour
{
    public static CameraSystem Instance;

    [Header("Main Cameras")]
    public Camera overviewCamera;
    public Camera followCamera;

    [Header("Follow Settings")]
    public Transform followTarget;
    public Vector3 followOffset = new Vector3(0f, 3f, -5f);
    public float followSpeed = 5f;
    public float lookAtSpeed = 3f;

    private bool followActive = true;

    void Awake()
    {
        if (Instance == null)
        {
            Instance = this;
            SetupCameras();
        }
        else
        {
            Destroy(gameObject);
        }
    }

    void SetupCameras()
    {
        // Проверка всех компонентов
        if (!CheckComponents()) return;

        // Начинаем с Follow камеры
        followActive = true;
        UpdateCameraStates();
        UpdateFollowCamera(true); // Мгновенное позиционирование
    }

    bool CheckComponents()
    {
        if (overviewCamera == null || followCamera == null || followTarget == null)
        {
            Debug.LogError("Не все компоненты камеры назначены в инспекторе!");
            enabled = false;
            return false;
        }
        return true;
    }

    void LateUpdate()
    {
        if (followActive)
        {
            UpdateFollowCamera();
        }
    }

    void UpdateFollowCamera(bool instant = false)
    {
        Vector3 targetPosition = followTarget.position + followOffset;
        Quaternion targetRotation = Quaternion.LookRotation(followTarget.position - targetPosition);

        if (instant)
        {
            followCamera.transform.position = targetPosition;
            followCamera.transform.rotation = targetRotation;
        }
        else
        {
            followCamera.transform.position = Vector3.Lerp(
                followCamera.transform.position,
                targetPosition,
                followSpeed * Time.deltaTime
            );
            followCamera.transform.rotation = Quaternion.Slerp(
                followCamera.transform.rotation,
                targetRotation,
                lookAtSpeed * Time.deltaTime
            );
        }
    }

    void UpdateCameraStates()
    {
        overviewCamera.gameObject.SetActive(!followActive);
        followCamera.gameObject.SetActive(followActive);
    }

    public void ToggleCamera()
    {
        if (!CheckComponents()) return;

        followActive = !followActive;
        UpdateCameraStates();

        if (followActive)
        {
            UpdateFollowCamera(true); // Мгновенное обновление при переключении
        }
    }
}