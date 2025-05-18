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
        Debug.Log("=== ������������� ������ ===");
        Debug.Log("Found obstacles: " + GameObject.FindGameObjectsWithTag("Obstacle").Length);

        path = new List<Vector3>();

        lineRenderer = GetComponent<LineRenderer>();
        lineRenderer.positionCount = 0;

        try
        {
            // 1. ������ ��� ��������
            sendUdpClient = new UdpClient(0);  // ��������� ��������� ����

            // 2. ������ ��� ������
            receiveUdpClient = new UdpClient(1235);
            receiveUdpClient.Client.ReceiveTimeout = 10000; // 10 ������

            Debug.Log($"����� �������� �: {((IPEndPoint)receiveUdpClient.Client.LocalEndPoint).Port}");

            // 3. ������ ������
            receiveThread = new Thread(ReceiveData);
            receiveThread.IsBackground = true;
            receiveThread.Start();

            // �������� ������
            //SendTargetToPython("30,0,0", "10,0,0,0.5");
            SendTargetToPython(
                $"{targetPosition.x},{targetPosition.y},{targetPosition.z}",
                GetObstaclesData());
        }
        catch (Exception e)
        {
            Debug.LogError($"������ �������������: {e}");
        }
    }

    void Update()
    {
        // ��������� �������� � ������� ������
        while (mainThreadActions.Count > 0)
        {
            mainThreadActions.Dequeue().Invoke();
        }

        // �������� ������ ���� ��������� Moving � ���� ����
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
                if (currentIndex >= path.Count) Debug.Log("����� ����������");
            }
        }
    }
    public void StartMovement()
    {
        if (path.Count == 0)
        {
            Debug.Log("���� �� �������. �������� ������ �� Python...");
            return;
        }

        state = MovementState.Moving;
        if (currentIndex >= path.Count) currentIndex = 0;
        Debug.Log($"����� ��������. ����� {currentIndex}/{path.Count}");
    }
    public void PauseMovement()
    {
        state = MovementState.Paused;
        Debug.Log("�������� ��������������");
    }
    public void StopMovement()
    {
        state = MovementState.Stopped;
        currentIndex = 0;
        if (path.Count > 0) transform.position = path[0];
        Debug.Log("�������� ����������� � ��������");
    }
    string GetObstaclesData()
    {
        var obstacles = GameObject.FindGameObjectsWithTag("Obstacle");
        if (obstacles.Length == 0)
        {
            Debug.Log("No obstacles found, returning empty array");
            return "[]";
        }

        // ������� ������ ��� �������� ������ � ������������
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

        // ���������� ��������������� �����-�������
        var wrapper = new ObstacleDataWrapper { obstacles = obstacleDataList };
        string json = JsonUtility.ToJson(wrapper);
        Debug.Log("Generated obstacles JSON: " + json);

        // ������� ������� ��� ������������� � Python
        json = json.Replace("{\"obstacles\":", "").TrimEnd('}');
        Debug.Log("Final JSON: " + json);
        return json;
    }

    // ����� ��� �������� ������ � �����������
    [System.Serializable]
    private class ObstacleData
    {
        public float x;
        public float y;
        public float z;
        public float radius;
    }

    // �����-������� ��� ���������� ������������ ������
    [System.Serializable]
    private class ObstacleDataWrapper
    {
        public List<ObstacleData> obstacles;
    }

    void ReceiveData()
    {
        Debug.Log("����� ������ ������ �������");
        IPEndPoint remoteEP = new IPEndPoint(IPAddress.Any, unityPort); // ���� ������� ����

        while (isRunning)
        {
            try
            {
                Debug.Log($"����� bound �: {((IPEndPoint)receiveUdpClient.Client.LocalEndPoint).Port}");
                Debug.Log("�������� ������...");
                byte[] data = receiveUdpClient.Receive(ref remoteEP);
                Debug.Log($"�������� {data.Length} ���� �� {remoteEP}");

                // ���������� ���������� �����
                int pointCount = BitConverter.ToInt32(data, 0);
                Debug.Log($"������ ���� {pointCount} �����");

                // �������� �������
                int expectedSize = 4 + pointCount * 12;
                if (data.Length != expectedSize)
                {
                    Debug.LogError($"�������������� �������! ��������� {expectedSize}, �������� {data.Length}");
                    continue;
                }

                // ��������� �����
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

                // ���������� � ������� ������
                lock (mainThreadActions)
                {
                    mainThreadActions.Enqueue(() =>
                    {
                        path = newPath;
                        currentIndex = 0;
                        lineRenderer.positionCount = path.Count;
                        lineRenderer.SetPositions(path.ToArray());
                        Debug.Log($"���������� ���� �� {path.Count} �����");

                        // ������������ � ������ �����
                        if (path.Count > 0)
                        {
                            transform.position = path[0];
                            Debug.Log($"��������� ������� �����������: {path[0]}");
                        }
                    });
                }
            }
            catch (SocketException e)
            {
                if (e.SocketErrorCode == SocketError.TimedOut || e.SocketErrorCode == SocketError.ConnectionReset)
                {
                    // ��� ���������� ��������� ��� UDP ��� ��������
                    continue;
                }
                if (isRunning) Debug.LogError($"������ ������: {e.Message}");
            }
            catch (Exception e)
            {
                Debug.LogError($"����������� ������: {e}");
            }
        }
    }

    void ProcessReceivedData(byte[] data)
    {
        if (data.Length < 16)
        {
            Debug.LogWarning($"������� ��������� �����: {data.Length} ����");
            return;
        }

        // ������ ���������� �����
        int pointCount = BitConverter.ToInt32(data, 0);
        Debug.Log($"��������� {pointCount} �����");

        List<Vector3> newPath = new List<Vector3>();

        // ������ ���������
        for (int i = 0; i < pointCount; i++)
        {
            int offset = 4 + i * 12;
            Vector3 point = new Vector3(
                BitConverter.ToSingle(data, offset),
                BitConverter.ToSingle(data, offset + 4),
                BitConverter.ToSingle(data, offset + 8)
            );
            newPath.Add(point);
            Debug.Log($"����� {i}: {point}");
        }

        // ��������� �������� � ������� �������� ������
        lock (mainThreadActions)
        {
            mainThreadActions.Enqueue(() =>
            {
                path = newPath;
                currentIndex = 0;
                UpdateLineRenderer();

                Debug.Log($"���������� ���������. ����� �����: {path.Count}");
                if (path.Count > 0)
                {
                    Debug.Log($"�������� �������� � �����: {path[0]}");
                    transform.position = path[0]; // �������� � ������ �����
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
            Debug.Log($"��������� ������: {message}");
        }
        catch (Exception e)
        {
            Debug.LogError($"������ ��������: {e}");
        }
    }

    void OnDestroy()
    {
        Debug.Log("��������� �������...");
        isRunning = false;

        receiveUdpClient?.Close();
        sendUdpClient?.Close();

        if (receiveThread != null && receiveThread.IsAlive)
            receiveThread.Join(500);

        Debug.Log("������� �����������");
    }

    void OnDrawGizmos()
    {
        if (path.Count == 0) return;

        // ������ ���� ����
        Gizmos.color = Color.blue;
        for (int i = 0; i < path.Count; i++)
        {
            Gizmos.DrawSphere(path[i], 0.2f);
            if (i > 0)
                Gizmos.DrawLine(path[i - 1], path[i]);
        }

        // ������� ����
        if (currentIndex < path.Count)
        {
            Gizmos.color = Color.green;
            Gizmos.DrawSphere(path[currentIndex], 0.3f);
        }
    }
}