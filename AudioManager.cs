using UnityEngine;

public class AudioManager : MonoBehaviour
{
    public static AudioManager Instance;
    private AudioListener listener;

    void Awake()
    {
        if (Instance == null)
        {
            Instance = this;
            listener = gameObject.AddComponent<AudioListener>();
            DontDestroyOnLoad(gameObject);
        }
        else Destroy(gameObject);
    }

    public void SetListenerTarget(Transform target)
    {
        transform.position = target.position;
        transform.rotation = target.rotation;
    }
}