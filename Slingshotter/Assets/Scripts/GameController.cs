using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEditor;

[System.Serializable]
public class Level
{
    [SerializeField] string name;
    public string Name => name;
    [SerializeField] int sceneIndex;
    public int SceneIndex => sceneIndex;
    [SerializeField] int nextLevelSceneIndex;
    public int NextLevelSceneIndex => nextLevelSceneIndex;
    [SerializeField] Vector3 playerSpawn;
    public Vector3 PlayerSpawn => playerSpawn;

}
public class GameController : MonoBehaviour
{
    static GameController instance;
    [SerializeField] List<GameObject> doNotDestroyList;
    [SerializeField] List<Level> levels;
    [SerializeField] new Transform camera;
    [SerializeField] Transform playerCharacter;
    [SerializeField] float playerDeathThresholdY;
    Level currentLevel;
    Vector3 cameraTarget;
    CharacterMovement playerMovement;

    [ExecuteInEditMode]
    void OnDrawGizmos()
    {
#if UNITY_EDITOR
        for (int i = 0; i < levels.Count; i++)
        {
            Gizmos.color = new Color(0f, 1f, 0f, 0.25f);
            Gizmos.DrawSphere(levels[i].PlayerSpawn, 0.25f + (0.1f * i));
        }
#endif
    }

    void Awake()
    {
        if (instance != null && instance != this)
        {
            Destroy(this);
            foreach (GameObject obj in doNotDestroyList)
            {
                Destroy(obj);
            }
        }
        else if (instance == null)
        {
            instance = this;
            DontDestroyOnLoad(this);
            foreach (GameObject obj in doNotDestroyList)
            {
                DontDestroyOnLoad(obj);
            }
        }

        SceneManager.sceneLoaded += OnLevelLoaded;
    }

    void Start()
    {
        if (playerCharacter != null)
            playerMovement = playerCharacter.GetComponent<CharacterMovement>();
    }

    void OnLevelLoaded(Scene scene, LoadSceneMode mode)
    {
        if (playerCharacter == null || playerMovement == null)
            return;

        if (currentLevel == null)
            return;

        playerMovement.ResetCharacter(currentLevel.PlayerSpawn);
    }

    void LoadLevel(Level level)
    {
        SceneManager.LoadScene(level.SceneIndex);
        currentLevel = level;
    }

    void Update()
    {
        CheckPlayerFell();

        // Debug!
        if (Input.GetKeyDown(KeyCode.Alpha1))
        {
            if (levels.Count > 0)
            {
                Level level = levels[0];
                Debug.Log("Loading level " + level.Name);
                LoadLevel(level);
            }
        }
        else if (Input.GetKeyDown(KeyCode.Alpha2))
        {
            if (levels.Count > 1)
            {
                Level level = levels[1];
                Debug.Log("Loading level " + level.Name);
                LoadLevel(level);
            }
        }
    }

    void FixedUpdate()
    {
        HandleCameraFollow();
    }

    void HandleCameraFollow()
    {
        Vector3 playerPos = playerCharacter.position;
        MoveCameraTowards(playerPos);
    }

    void MoveCameraTowards(Vector3 target)
    {
        Vector3 cameraPos = camera.position;
        // Vector3 playerPos = playerCharacter.position;

        Vector3 newPos = Vector3.Lerp(cameraPos, target, 1f * Time.fixedDeltaTime);

        camera.position = newPos;
    }

    void CheckPlayerFell()
    {
        if (playerCharacter.position.y < playerDeathThresholdY)
        {
            Debug.Log("OH NO, THE PLAYER DIED.");
            playerMovement.ResetCharacter(currentLevel.PlayerSpawn);
        }
    }
}
