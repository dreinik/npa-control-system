using UnityEngine;
using UnityEngine.UI;

public class CameraSwitch : MonoBehaviour
{
    [Header("Main Cameras")]
    public GameObject followCameraObj; // ������ � Follow �������
    public GameObject overviewCameraObj; // ������ � Overview �������

    private Camera followCam;
    private Camera overviewCam;
    private OverviewCamera overviewCameraScript;

    void Start()
    {
        // �������� ���������� �����
        followCam = followCameraObj.GetComponent<Camera>();
        overviewCam = overviewCameraObj.GetComponent<Camera>();
        overviewCameraScript = overviewCameraObj.GetComponent<OverviewCamera>();

        // �������� � Follow ������
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
        // ��������/��������� ���� ������ ������
        cameraObj.SetActive(state);

        // ������������� �������� ��������� Camera
        var cam = cameraObj.GetComponent<Camera>();
        if (cam != null)
        {
            cam.enabled = state;
        }
    }
}