using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System.IO;

public class TrajectoryLoader : MonoBehaviour
{
    public string csvFileName = "trajectory.csv";
    public float playbackSpeed = 1.0f; // Скорость воспроизведения
    public LayerMask obstacleLayer;

    private string csvPath;
    private List<Vector3> positions = new List<Vector3>();
    private int currentIndex = 0;
    private float timer = 0;
    private LineRenderer lineRenderer;

    void Start()
    {
        csvPath = Path.Combine(Application.streamingAssetsPath, csvFileName);
        LoadCSV();

        lineRenderer = GetComponent<LineRenderer>();
        lineRenderer.positionCount = positions.Count;
        lineRenderer.SetPositions(positions.ToArray());

        obstacleLayer = LayerMask.GetMask("Obstacle");
    }

    void LoadCSV()
    {
        if (!File.Exists(csvPath))
        {
            Debug.LogError("Файл не найден: " + csvPath);
            return;
        }

        string[] lines = File.ReadAllLines(csvPath);
        for (int i = 1; i < lines.Length; i++) // Пропускаем заголовок
        {
            string[] values = lines[i].Split(',');
            if (values.Length >= 4)
            {
                float x = float.Parse(values[1].Trim(), System.Globalization.NumberStyles.Float, System.Globalization.CultureInfo.InvariantCulture);
                float y = float.Parse(values[2].Trim(), System.Globalization.NumberStyles.Float, System.Globalization.CultureInfo.InvariantCulture);
                float z = float.Parse(values[3].Trim(), System.Globalization.NumberStyles.Float, System.Globalization.CultureInfo.InvariantCulture);
                positions.Add(new Vector3(x, y, z));
            }
        }
    }

    void Update()
    {
        if (currentIndex < positions.Count)
        {
            timer += Time.deltaTime * playbackSpeed;
            if (timer >= 0.1f) // Задержка между точками (можно настроить)
            {
                transform.position = positions[currentIndex];
                currentIndex++;
                timer = 0;
            }
        }
    }

    public void SkipDangerousPoints(Vector3 obstacleCenter, float radius)
    {
        for (int i = 0; i < positions.Count; i++)
        {
            if (Vector3.Distance(positions[i], obstacleCenter) < radius)
            {
                positions.RemoveAt(i);
                i--; // Корректируем индекс после удаления
            }
        }
        lineRenderer.positionCount = positions.Count; // Обновляем LineRenderer
    }
}
