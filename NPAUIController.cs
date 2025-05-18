using UnityEngine;
using UnityEngine.UI;
using TMPro;

public class NPAUIController : MonoBehaviour
{
    [Header("Camera System")]
    public CameraSystem cameraSystem;

    [Header("UI Elements")]
    public Button startPauseButton;
    public Button cameraSwitchButton;
    public Button resetButton;
    public TextMeshProUGUI statusText;

    [Header("References")]
    public NPAController npaController;

    private bool isRunning = false;

    void Start()
    {
        // Проверка ссылок
        if (npaController == null)
        {
            Debug.LogError("NPAController не назначен!");
            return;
        }

        // Настройка кнопок
        startPauseButton.onClick.AddListener(ToggleStartPause);
        cameraSwitchButton.onClick.AddListener(() => {
            cameraSystem.ToggleCamera();
        });
        resetButton.onClick.AddListener(ResetSimulation);

        UpdateUI();
    }

    void ToggleStartPause()
    {
        isRunning = !isRunning;

        if (isRunning) npaController.StartMovement();
        else npaController.PauseMovement();

        UpdateUI();
    }

    public void SwitchCamera()
    {
        if (CameraSystem.Instance != null)
        {
            CameraSystem.Instance.ToggleCamera();
            Debug.Log("Кнопка переключения камеры нажата"); // Добавляем лог
        }
        else
        {
            Debug.LogError("CameraSystem.Instance не найден!");
        }
    }

    void ResetSimulation()
    {
        npaController.StopMovement();
        isRunning = false;
        UpdateUI();
    }

    void UpdateUI()
    {
        statusText.text = isRunning ? "RUNNING" : "PAUSED";
        startPauseButton.GetComponentInChildren<TextMeshProUGUI>().text =
            isRunning ? "PAUSE" : "START";
    }
}