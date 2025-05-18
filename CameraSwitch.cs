using UnityEngine;
using UnityEngine.UI;

public class CameraSwitch : MonoBehaviour
{
    [Header("Main Cameras")]
    public GameObject followCameraObj; // Объект с Follow камерой
    public GameObject overviewCameraObj; // Объект с Overview камерой

    private Camera followCam;
    private Camera overviewCam;
    private OverviewCamera overviewCameraScript;

    void Start()
    {
        // Получаем компоненты камер
        followCam = followCameraObj.GetComponent<Camera>();
        overviewCam = overviewCameraObj.GetComponent<Camera>();
        overviewCameraScript = overviewCameraObj.GetComponent<OverviewCamera>();

        // Начинаем с Follow камеры
        SetCameraState(followCameraObj, true);
        SetCameraState(overviewCameraObj, false);
    }

    public void SwitchCamera()
    {
        bool followActive = !followCameraObj.activeSelf;

        SetCameraState(followCameraObj, followActive);
        SetCameraState(overviewCameraObj, !followActive);

        if (!followActive)
        {
            overviewCameraScript.ResetCameraPosition();
        }
    }

    private void SetCameraState(GameObject cameraObj, bool state)
    {
        // Включаем/выключаем весь объект камеры
        cameraObj.SetActive(state);

        // Дополнительно включаем компонент Camera
        var cam = cameraObj.GetComponent<Camera>();
        if (cam != null)
        {
            cam.enabled = state;
        }
    }
}