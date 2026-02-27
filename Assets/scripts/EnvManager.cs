using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using UnityEngine.InputSystem;
using System.Collections;  
using System.Collections.Generic;
public class EnvManager : MonoBehaviour
{
    public GameObject victimPrefab;
    public GameObject agent;
    float targetDistance = 0.0f;
    int obstacleCount = 0;
    public int victimCount = 5;
    public List<GameObject> activeVictims = new List<GameObject>();
    public Transform[] obstacles;
    private Rigidbody2D droneRb;
    public bool isInitializing = true;

    private void Start()
    {
        droneRb = agent.GetComponent<Rigidbody2D>();
    }

    public void ResetEnvironment()
    {   
        Debug.Log("Resetting Environment");
        isInitializing = true;
        InitCurriculumEnv();
        DeactivateAgent();
        SpawnVictims(victimCount);
        SpawnObstacles(obstacleCount);
        StartCoroutine(LaunchDroneAfterGoalsSettle());
    }

    void InitCurriculumEnv(){
        var envParams = Academy.Instance.EnvironmentParameters;
        targetDistance = envParams.GetWithDefault("target_distance", 10.0f);
        obstacleCount = (int)envParams.GetWithDefault("obstacle_count", 6f);
    }

    Vector2 GetRandomTargetPosition(){
        
        // Center of arena
        Vector2 arenaCenter = new Vector2(0f, 0f); // Adjust to your arena center
        
        // Random angle (full 360 degrees)
        float angle = Random.Range(0f, Mathf.PI * 2f);
        
        // Distance based on curriculum (but ensure minimum spread)
        float minSpread = 3f;
        float maxSpread = Mathf.Min(targetDistance * 0.8f, 16f); // Scale with curriculum
        float distance = Random.Range(minSpread, maxSpread);
        
        float offsetX = Mathf.Cos(angle) * distance;
        float offsetY = Mathf.Sin(angle) * distance * 0.3f; // Less vertical spread
        
        float targetX = arenaCenter.x + offsetX;
        float targetY = arenaCenter.y + offsetY;
        
        // Clamp to valid area
        targetX = Mathf.Clamp(targetX, -14f, 14f);
        targetY = Mathf.Clamp(targetY, -2f, 3f); // Keep victims on/near ground
        
        return new Vector2(targetX, targetY);
    }

    void DeactivateAgent(){
       agent.transform.position = new Vector3(0, -100, 0); // Move off-screen
       droneRb.Sleep();
    }

    void SpawnVictims(int count)
    {
        foreach (GameObject victim in activeVictims)
        {
           if(victim != null) Destroy(victim);
        }
        activeVictims.Clear();
        for (int i = 0; i < count; i++)
        {
            Vector2 pos = GetRandomTargetPosition();
            GameObject victim = Instantiate(victimPrefab, pos, Quaternion.identity);
            activeVictims.Add(victim);
        }
    }

    void SpawnObstacles(int count)
    {
        // Setup obstacles
        int maxObs = Mathf.Min(count, obstacles.Length);
        for (int i = 0; i < maxObs; i++){
            int randomIndex = Random.Range(i, obstacles.Length);
            Vector2 pos1 = obstacles[i].localPosition;
            Vector2 pos2 = obstacles[randomIndex].localPosition;
            obstacles[i].localPosition = new Vector2(pos2.x, pos1.y);
            obstacles[randomIndex].localPosition = new Vector2(pos1.x, pos2.y);
            obstacles[i].gameObject.SetActive(true);
        }
        
        // Deactivate unused obstacles
        for (int i = maxObs; i < obstacles.Length; i++){
            obstacles[i].gameObject.SetActive(false);
        }
    }

    IEnumerator LaunchDroneAfterGoalsSettle()
    {
        float timeout = 5f;
        float elapsed = 0f;

        while (elapsed < timeout)
        {
            bool allSettled = true;

            foreach (GameObject victim in activeVictims)
            {
                Rigidbody2D victimRb = victim.GetComponent<Rigidbody2D>();
                if (victimRb != null &&
                    (!victimRb.IsSleeping() || victimRb.linearVelocity.magnitude > 0.05f))
                {
                    allSettled = false;
                    break;
                }
            }

            if (allSettled)
                break;

            yield return new WaitForFixedUpdate();
            elapsed += Time.fixedDeltaTime;
        }

        yield return new WaitForFixedUpdate();

        SpawnAgent();
}

    void SpawnAgent()
    {
        droneRb.WakeUp();
        agent.transform.localPosition = new Vector2(Random.Range(-7f, 7f), 4f);
        droneRb.SetRotation(0f);
        droneRb.linearVelocity = Vector2.zero;
        droneRb.angularVelocity = 0f;
        droneRb.linearVelocity = Vector2.zero;
        droneRb.angularVelocity = 0f;
        isInitializing = false;

        agent.GetComponent<Agent2D>().PostEnvironmentReset();
    }
}
