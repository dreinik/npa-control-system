using UnityEngine;
using System;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using System.Collections.Generic;
using System.Collections;

public class NPAController : MonoBehaviour
{
    [Header("Control Settings")]

    [Header("UDP Settings")]
    public string pythonIP = "127.0.0.1";
    public int pythonPort = 1234;
    public int unityPort = 1235;
    public float movementSpeed = 5.0f;
    public Vector3 targetPosition = new Vector3(30, 0, 0);

    private UdpClient sendUdpClient;
    private UdpClient receiveUdpClient;
    private Thread receiveThread;
    [HideInInspector] public List<Vector3> path = new List<Vector3>();
    private int currentIndex = 0;
    private bool isRunning = true;
    private LineRenderer lineRenderer;
    private readonly Queue<Action> mainThreadActions = new Queue<Action>();

    private enum MovementState { Stopped, Paused, Moving }
    private MovementState state = MovementState.Stopped;

    public static NPAController Instance;
    void Awake()
    {
        if (Instance == null)
        {
            Instance = this;
        }
        else
        {
            Destroy(gameObject);
        }
    }

        void Start()
    {
        Debug.Log("=== ИНИЦИАЛИЗАЦИЯ НАЧАТА ===");
        Debug.Log("Found obstacles: " + GameObject.FindGameObjectsWithTag("Obstacle").Length);

        path = new List<Vector3>();

        lineRenderer = GetComponent<LineRenderer>();
        lineRenderer.positionCount = 0;

        try
        {
            // 1. Клиент для отправки
            sendUdpClient = new UdpClient(0);  // Случайный свободный порт

            // 2. Клиент для приема
            receiveUdpClient = new UdpClient(1235);
            receiveUdpClient.Client.ReceiveTimeout = 10000; // 10 секунд

            Debug.Log($"Сокет привязан к: {((IPEndPoint)receiveUdpClient.Client.LocalEndPoint).Port}");

            // 3. Запуск потока
            receiveThread = new Thread(ReceiveData);
            receiveThread.IsBackground = true;
            receiveThread.Start();

            // Тестовый запрос
            //SendTargetToPython("30,0,0", "10,0,0,0.5");
            SendTargetToPython(
                $"{targetPosition.x},{targetPosition.y},{targetPosition.z}",
                GetObstaclesData());
        }
        catch (Exception e)
        {
            Debug.LogError($"Ошибка инициализации: {e}");
        }
    }

    void Update()
    {
        // Обработка действий в главном потоке
        while (mainThreadActions.Count > 0)
        {
            mainThreadActions.Dequeue().Invoke();
        }

        // Движение только если состояние Moving и есть путь
        if (state == MovementState.Moving && path.Count > 0 && currentIndex < path.Count)
        {
            transform.position = Vector3.MoveTowards(
                transform.position,
                path[currentIndex],
                movementSpeed * Time.deltaTime
            );

            if (Vector3.Distance(transform.position, path[currentIndex]) < 0.1f)
            {
                currentIndex++;
                if (currentIndex >= path.Count) Debug.Log("Конец траектории");
            }
        }
    }
    public void StartMovement()
    {
        if (path.Count == 0)
        {
            Debug.Log("Путь не получен. Ожидание данных от Python...");
            return;
        }

        state = MovementState.Moving;
        if (currentIndex >= path.Count) currentIndex = 0;
        Debug.Log($"Старт движения. Точка {currentIndex}/{path.Count}");
    }
    public void PauseMovement()
    {
        state = MovementState.Paused;
        Debug.Log("Движение приостановлено");
    }
    public void StopMovement()
    {
        state = MovementState.Stopped;
        currentIndex = 0;
        if (path.Count > 0) transform.position = path[0];
        Debug.Log("Движение остановлено и сброшено");
    }
    string GetObstaclesData()
    {
        var obstacles = GameObject.FindGameObjectsWithTag("Obstacle");
        if (obstacles.Length == 0)
        {
            Debug.Log("No obstacles found, returning empty array");
            return "[]";
        }

        // Создаем список для хранения данных о препятствиях
        var obstacleDataList = new List<ObstacleData>();

        foreach (var obs in obstacles)
        {
            if (obs == null) continue;

            var collider = obs.GetComponent<Collider>();
            if (collider == null)
            {
                Debug.LogWarning($"Object {obs.name} has no collider");
                continue;
            }

            var pos = obs.transform.position;
            float radius = 0;

            if (collider is SphereCollider sphere)
            {
                radius = sphere.radius;
            }

            obstacleDataList.Add(new ObstacleData
            {
                x = pos.x,
                y = pos.y,
                z = pos.z,
                radius = radius
            });
        }

        // Используем вспомогательный класс-обертку
        var wrapper = new ObstacleDataWrapper { obstacles = obstacleDataList };
        string json = JsonUtility.ToJson(wrapper);
        Debug.Log("Generated obstacles JSON: " + json);

        // Удаляем обертку для совместимости с Python
        json = json.Replace("{\"obstacles\":", "").TrimEnd('}');
        Debug.Log("Final JSON: " + json);
        return json;
    }

    // Класс для хранения данных о препятствии
    [System.Serializable]
    private class ObstacleData
    {
        public float x;
        public float y;
        public float z;
        public float radius;
    }

    // Класс-обертка для корректной сериализации списка
    [System.Serializable]
    private class ObstacleDataWrapper
    {
        public List<ObstacleData> obstacles;
    }

    void ReceiveData()
    {
        Debug.Log("Поток приема данных запущен");
        IPEndPoint remoteEP = new IPEndPoint(IPAddress.Any, unityPort); // Явно укажите порт

        while (isRunning)
        {
            try
            {
                Debug.Log($"Сокет bound к: {((IPEndPoint)receiveUdpClient.Client.LocalEndPoint).Port}");
                Debug.Log("Ожидание данных...");
                byte[] data = receiveUdpClient.Receive(ref remoteEP);
                Debug.Log($"Получено {data.Length} байт от {remoteEP}");

                // Декодируем количество точек
                int pointCount = BitConverter.ToInt32(data, 0);
                Debug.Log($"Должно быть {pointCount} точек");

                // Проверка размера
                int expectedSize = 4 + pointCount * 12;
                if (data.Length != expectedSize)
                {
                    Debug.LogError($"Несоответствие размера! Ожидалось {expectedSize}, получено {data.Length}");
                    continue;
                }

                // Обработка точек
                List<Vector3> newPath = new List<Vector3>();
                for (int i = 0; i < pointCount; i++)
                {
                    int offset = 4 + i * 12;
                    Vector3 point = new Vector3(
                        BitConverter.ToSingle(data, offset),
                        BitConverter.ToSingle(data, offset + 4),
                        BitConverter.ToSingle(data, offset + 8)
                    );
                    newPath.Add(point);
                }

                // Обновление в главном потоке
                lock (mainThreadActions)
                {
                    mainThreadActions.Enqueue(() =>
                    {
                        path = newPath;
                        currentIndex = 0;
                        lineRenderer.positionCount = path.Count;
                        lineRenderer.SetPositions(path.ToArray());
                        Debug.Log($"Установлен путь из {path.Count} точек");

                        // Телепортация к первой точке
                        if (path.Count > 0)
                        {
                            transform.position = path[0];
                            Debug.Log($"Начальная позиция установлена: {path[0]}");
                        }
                    });
                }
            }
            catch (SocketException e)
            {
                if (e.SocketErrorCode == SocketError.TimedOut || e.SocketErrorCode == SocketError.ConnectionReset)
                {
                    // Это нормальное поведение для UDP при ожидании
                    continue;
                }
                if (isRunning) Debug.LogError($"Ошибка сокета: {e.Message}");
            }
            catch (Exception e)
            {
                Debug.LogError($"Критическая ошибка: {e}");
            }
        }
    }

    void ProcessReceivedData(byte[] data)
    {
        if (data.Length < 16)
        {
            Debug.LogWarning($"Слишком маленький пакет: {data.Length} байт");
            return;
        }

        // Чтение количества точек
        int pointCount = BitConverter.ToInt32(data, 0);
        Debug.Log($"Ожидается {pointCount} точек");

        List<Vector3> newPath = new List<Vector3>();

        // Чтение координат
        for (int i = 0; i < pointCount; i++)
        {
            int offset = 4 + i * 12;
            Vector3 point = new Vector3(
                BitConverter.ToSingle(data, offset),
                BitConverter.ToSingle(data, offset + 4),
                BitConverter.ToSingle(data, offset + 8)
            );
            newPath.Add(point);
            Debug.Log($"Точка {i}: {point}");
        }

        // Добавляем действие в очередь главного потока
        lock (mainThreadActions)
        {
            mainThreadActions.Enqueue(() =>
            {
                path = newPath;
                currentIndex = 0;
                UpdateLineRenderer();

                Debug.Log($"Траектория обновлена. Всего точек: {path.Count}");
                if (path.Count > 0)
                {
                    Debug.Log($"Начинаем движение к точке: {path[0]}");
                    transform.position = path[0]; // Начинаем с первой точки
                }
            });
        }
    }

    void UpdateLineRenderer()
    {
        lineRenderer.positionCount = path.Count;
        lineRenderer.SetPositions(path.ToArray());
    }

    public void SendTargetToPython(string target, string obstacles)
    {
        try
        {
            string message = $"{target};{obstacles}";
            byte[] data = Encoding.UTF8.GetBytes(message);
            sendUdpClient.Send(data, data.Length, pythonIP, pythonPort);
            Debug.Log($"Отправлен запрос: {message}");
        }
        catch (Exception e)
        {
            Debug.LogError($"Ошибка отправки: {e}");
        }
    }

    void OnDestroy()
    {
        Debug.Log("Остановка системы...");
        isRunning = false;

        receiveUdpClient?.Close();
        sendUdpClient?.Close();

        if (receiveThread != null && receiveThread.IsAlive)
            receiveThread.Join(500);

        Debug.Log("Система остановлена");
    }

    void OnDrawGizmos()
    {
        if (path.Count == 0) return;

        // Рисуем весь путь
        Gizmos.color = Color.blue;
        for (int i = 0; i < path.Count; i++)
        {
            Gizmos.DrawSphere(path[i], 0.2f);
            if (i > 0)
                Gizmos.DrawLine(path[i - 1], path[i]);
        }

        // Текущая цель
        if (currentIndex < path.Count)
        {
            Gizmos.color = Color.green;
            Gizmos.DrawSphere(path[currentIndex], 0.3f);
        }
    }
}