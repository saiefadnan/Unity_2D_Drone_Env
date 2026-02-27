using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using UnityEngine.InputSystem;
using System.Collections;  
using System.Collections.Generic;

public class Agent2D : Agent
{
    [Header("Agent Configuration")]
    public bool isManualControl = false;
    public RayPerceptionSensorComponent2D raySensor;
    public AudioSource targetReached;
    public AudioSource droneHum;
    public int maxStepCount = 2000;
    public EnvManager envManager;

    private Transform[] goals;

    // Episode tracking
    Vector2 lastPos;
    float shortestPath = 0f;
    float distanceTraveled = 0f;
    int goalsReached = 0;
    int groundCollision = 0;
    int targetIndex = 0;
    int StepCnt = 0;
    
    // Physics
    Rigidbody2D rb;
    float deltaX = 0.0f;
    float targetY = 0.0f;
    float softLandingThreshold = 1.5f;
    float hardLandingThreshold = 4f;
    
    // Reward shaping
    float previousDistance = Mathf.Infinity;
    private float closestDistanceEver = Mathf.Infinity; // Anti-oscillation
    private int backtrackCounter = 0;
    
    // Exploration system
    private HashSet<Vector2Int> visitedCells = new HashSet<Vector2Int>();
    private int hoverCounter = 0;
    private Vector2 lastHoverCheck;
    private const float GRID_CELL_SIZE = 2.5f;
    private string endReason = "start";

    public override void Initialize(){
        rb = GetComponent<Rigidbody2D>();
    }
    
    
    float DistanceToTarget(){
        if (!goals[targetIndex].gameObject.activeSelf) return 0f;
        return Vector2.Distance(goals[targetIndex].localPosition, transform.localPosition);
    }
    
    void calculateShortestPath(){
        shortestPath += deltaX;
        int k = targetIndex;
        Dictionary<int, bool> visited = new Dictionary<int, bool>();
        visited[k] = true;
        
        while(visited.Count < goals.Length){
            float nearestDist = Mathf.Infinity;
            int nearestIndex = -1;
            
            for(int i = 0; i < goals.Length; i++){
                if(!visited.ContainsKey(i) && i != k){
                    float dist = Vector2.Distance(goals[i].localPosition, goals[k].localPosition);
                    if(dist < nearestDist){
                        nearestDist = dist;
                        nearestIndex = i;
                    }
                }
            }
            
            if(nearestIndex != -1){
                shortestPath += nearestDist;
                visited[nearestIndex] = true;
                k = nearestIndex;
            }
        }
    }
    
    float GetNearestDistance(Vector3 source){
        float newDeltaX = Mathf.Infinity;
        foreach (Transform goal in goals){
            float dist = Vector2.Distance(goal.localPosition, source);
            if (goal.gameObject.activeSelf && dist < newDeltaX){
                newDeltaX = dist;
                targetIndex = System.Array.IndexOf(goals, goal);
            }
        }
        return newDeltaX;
    }
    
    public override void OnEpisodeBegin(){
        Debug.Log(endReason);
        envManager.ResetEnvironment();

        // Reset tracking variables
        targetY = transform.localPosition.y;
        shortestPath = 0f;
        lastPos = transform.localPosition;
        distanceTraveled = 0f;
        StepCnt = 0;
        groundCollision = 0;
        
        // Reset exploration systems
        visitedCells.Clear();
        hoverCounter = 0;
        lastHoverCheck = Vector2.zero;
        closestDistanceEver = Mathf.Infinity;
        backtrackCounter = 0;
        goalsReached = 0;
        
    }

    public void PostEnvironmentReset()
    {
        goals = envManager.activeVictims
            .ConvertAll(v => v.transform)
            .ToArray();

        deltaX = GetNearestDistance(transform.localPosition);
        calculateShortestPath();
        previousDistance = DistanceToTarget();
        closestDistanceEver = previousDistance;
        lastHoverCheck = transform.localPosition;
    }

    public override void CollectObservations(VectorSensor sensor){
        if(envManager.isInitializing) return; // Skip observations during initialization
        // Agent state (6 values)
        sensor.AddObservation(rb.linearVelocity / 10f); // 2 values
        sensor.AddObservation(rb.angularVelocity / 180f); // 1 value
        sensor.AddObservation(transform.localPosition / 16f); // 2 values
        // Agent orientation
        float normalizedRotation = transform.eulerAngles.z;
        if (normalizedRotation > 180) normalizedRotation -= 360;
        sensor.AddObservation(normalizedRotation / 180f); // 1 value
        // Task context for LSTM (1 value)
        sensor.AddObservation(goalsReached / (float)goals.Length); // Mission progress
    }

    void Update(){
        // Check if agent fell off bounds
        if (transform.localPosition.y > 8f || 
            transform.localPosition.x < -16f || transform.localPosition.x > 16f){
            AddReward(-10f);
            endReason = "out_of_bounds";
            EndEpisode();
        }
        // Draw ray sensor debug visualization
        if (raySensor != null){
            DrawRaySensorDebug();
        }
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        if(envManager.isInitializing) return; // Skip actions during initialization
        StepCnt++;
        if (StepCnt > maxStepCount){
            AddReward(-2f); // Penalty for timeout
            endReason = "timeout";
            EndEpisode();
            return;
        }

        // Apply drone control forces
        float forceX = actions.ContinuousActions[0];
        float forceY = actions.ContinuousActions[1];
        float torque = actions.ContinuousActions[2];
        rb.AddForce(new Vector2(forceX, forceY) * 10f, ForceMode2D.Force);
        rb.AddTorque(torque * 0.5f);

        if (isManualControl){
            previousDistance = DistanceToTarget();
            lastPos = transform.localPosition;
            return;
        }
        
        // ========== STABILITY REWARDS ==========
        float angleZ = transform.eulerAngles.z;
        float normalizedZRotation = angleZ > 180 ? angleZ - 360 : angleZ;
        float uprightReward = 1.0f - (Mathf.Abs(normalizedZRotation) / 180.0f);
        
        if (Mathf.Abs(normalizedZRotation) > 72f){
            AddReward(-0.02f);
            if (Mathf.Abs(normalizedZRotation) > 85f){
                AddReward(-3.0f);
                endReason = "unstable";
                EndEpisode();
                return;
            }
        }

        // ========== ANTI-OSCILLATION PROGRESS REWARD ==========
        float currentDistance = DistanceToTarget();
        
        if (currentDistance > 0f){
            // Only reward when beating personal record distance
            if (currentDistance < closestDistanceEver - 0.05f){ // Small threshold to avoid noise
                float improvement = closestDistanceEver - currentDistance;
                AddReward(Mathf.Clamp(improvement * 5f, 0f, 0.15f));
                closestDistanceEver = currentDistance;
                backtrackCounter = 0;
            }
            // Penalize moving away from target
            else if (currentDistance > previousDistance + 0.1f){
                AddReward(-0.015f);
                backtrackCounter++;
                
                // Strong penalty for persistent oscillation
                if (backtrackCounter > 20){
                    AddReward(-0.05f);
                }
            }
            else {
                // Gradually decay backtrack counter
                backtrackCounter = Mathf.Max(0, backtrackCounter - 1);
            }
            
            previousDistance = currentDistance;
        }
        
        // ========== EXPLORATION SYSTEM ==========
        // Grid-based exploration reward
        Vector2Int currentCell = new Vector2Int(
            Mathf.RoundToInt(transform.localPosition.x / GRID_CELL_SIZE),
            Mathf.RoundToInt(transform.localPosition.y / GRID_CELL_SIZE)
        );
        
        if (!visitedCells.Contains(currentCell)){
            visitedCells.Add(currentCell);
            AddReward(0.05f); // Reward for exploring new areas
        }
        
        // Anti-hovering system - check every 10 steps
        if (StepCnt % 10 == 0){
            float movement = Vector2.Distance(transform.localPosition, lastHoverCheck);
            
            if (movement < 0.1f){ // Barely moved in 10 steps
                hoverCounter++;
                if (hoverCounter > 6){ // 60 steps (~3 seconds) of hovering
                    AddReward(-0.05f);
                }
                if (hoverCounter > 12){ // 120 steps (~6 seconds)
                    AddReward(-0.1f);
                }
            } else {
                hoverCounter = 0;
            }
            
            lastHoverCheck = transform.localPosition;
        }

        // ========== MOVEMENT TRACKING ==========
        float newDist = Vector2.Distance(transform.localPosition, lastPos);
        if (newDist < 0.001f){
            AddReward(-0.01f); // Penalty for not moving
        }
        distanceTraveled += newDist;
        lastPos = transform.localPosition;
        
        // Time/energy cost
        AddReward(-0.0005f);
        
        // ========== STATS RECORDING ==========
        var recorder = Academy.Instance.StatsRecorder;
        recorder.Add("AngleStability", uprightReward);
        recorder.Add("ExploredCells", visitedCells.Count);
        recorder.Add("ClosestEver", closestDistanceEver);
        recorder.Add("BacktrackCount", backtrackCounter);
    }
    
    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var act = actionsOut.ContinuousActions;
        
        // Horizontal control
        float horiz = 0f;
        if (Keyboard.current.aKey.isPressed) horiz = -1f;
        else if (Keyboard.current.dKey.isPressed) horiz = 1f;
        
        // Vertical control
        float vert = 0f;
        if (Keyboard.current.wKey.isPressed) vert = 1f;
        else if (Keyboard.current.sKey.isPressed) vert = -1f;
        
        // Rotation control
        float torque = 0f;
        if (Keyboard.current.qKey.isPressed) torque = 1f;
        else if (Keyboard.current.eKey.isPressed) torque = -1f;
        
        act[0] = horiz;
        act[1] = vert;
        act[2] = torque;
    }

    void OnCollisionEnter2D(Collision2D other){
        if (other.gameObject.CompareTag("Ground")){
            groundCollision++;
            float impactSpeed = other.relativeVelocity.magnitude;
            
            if (1 < impactSpeed && impactSpeed < softLandingThreshold){
                AddReward(1f);
                Debug.Log("Soft landing.");
            }
            else if (impactSpeed >= hardLandingThreshold){
                AddReward(-5f);
                endReason = "hard_crash";
                EndEpisode();
                Debug.Log("Hard crash.");
                return;
            }
            else {
                AddReward(-0.2f);
                Debug.Log("Moderate landing.");
            }
        }
        
        if (other.gameObject.CompareTag("Obstacle")){
            AddReward(-5f);
            endReason = "crashed_into_obstacle";
            EndEpisode();
            return;
        }
    }

    void OnTriggerEnter2D(Collider2D other){
        if (other.CompareTag("Victim")){
            goalsReached++;
            
            // Calculate rewards
            float baseReward = 8.0f;
            float timeBonus = Mathf.Max(0f, (maxStepCount - StepCnt) / (float)maxStepCount) * 2f;
            
            if (System.Array.IndexOf(goals, other.transform) == targetIndex){
                AddReward(baseReward + timeBonus);
                Debug.Log($"Found target victim! Reward: {baseReward + timeBonus:F2}");
            }
            else {
                AddReward(baseReward * 0.7f + timeBonus);
                Debug.Log($"Found victim! Reward: {(baseReward * 0.7f + timeBonus):F2}");
            }
            
            targetReached?.Play();
            other.gameObject.SetActive(false);
        }
        
        // Check if all victims found
        if (goalsReached == goals.Length){
            float efficiency = shortestPath / Mathf.Max(distanceTraveled, 0.001f);
            float completionBonus = 15f;
            float efficiencyBonus = efficiency * 10f;
            
            AddReward(completionBonus + efficiencyBonus);
            
            var recorder = Academy.Instance.StatsRecorder;
            recorder.Add("Efficiency", efficiency);
            recorder.Add("CompletionTime", StepCnt);
            
            Debug.Log($"Mission complete! Total reward: {GetCumulativeReward():F2}");
            endReason = "Completion";
            EndEpisode();
        }
        else {
            // Update to next nearest target
            deltaX = GetNearestDistance(transform.localPosition);
            previousDistance = DistanceToTarget();
            closestDistanceEver = previousDistance; // Reset for new target
            backtrackCounter = 0;
        }
    }
    
    void DrawRaySensorDebug(){
        if (raySensor == null) return;

        float rayLength = raySensor.RayLength;
        int raysPerSide = raySensor.RaysPerDirection;
        float maxAngle = raySensor.MaxRayDegrees;
        int totalRays = raysPerSide * 2 + 1;

        for (int i = 0; i < totalRays; i++){
            float angle = (i - raysPerSide) * (maxAngle / raysPerSide);
            Vector3 dir = Quaternion.Euler(0f, 0f, angle) * raySensor.transform.up;
            Vector3 startPos = raySensor.transform.position;

            LayerMask layerMask = raySensor.GetComponent<RayPerceptionSensorComponent2D>().RayLayerMask;
            RaycastHit2D hit = Physics2D.Raycast(startPos, dir, rayLength, layerMask);

            Color rayColor = Color.red;
            
            if (hit.collider != null && hit.collider.gameObject != gameObject){
                string[] detectableTags = raySensor.DetectableTags.ToArray();
                
                if (System.Array.IndexOf(detectableTags, hit.collider.tag) >= 0){
                    if (hit.collider.CompareTag("Victim")){
                        rayColor = Color.green;
                    }
                    else if (hit.collider.CompareTag("Ground") || hit.collider.CompareTag("Obstacle")){
                        rayColor = Color.yellow;
                    }
                    else {
                        rayColor = Color.blue;
                    }
                }
            }
            Debug.DrawLine(startPos, startPos + dir * rayLength, rayColor);
        }
    }
    
    void OnGUI(){
        GUIStyle style = new GUIStyle();
        style.fontSize = 20;
        style.normal.textColor = Color.white;
        
        GUI.Label(new Rect(15, 25, 300, 30), 
                $"Reward: {GetCumulativeReward():F2}", style);
        GUI.Label(new Rect(15, 60, 300, 30), 
                $"Dist: {(envManager.isInitializing ? 0 : DistanceToTarget()):F1} | Record: {closestDistanceEver:F1}", style);
        GUI.Label(new Rect(15, 95, 300, 30), 
                $"Explored: {visitedCells.Count} | Hover: {hoverCounter}", style);
    }

    void RecordStats(){
        var recorder = Academy.Instance.StatsRecorder;
        recorder.Add("EpisodeLength", StepCnt);
        recorder.Add("TargetsFound", goalsReached);
        recorder.Add("GroundCollision", groundCollision);
        recorder.Add("PathEfficiency", shortestPath / Mathf.Max(distanceTraveled, 0.001f));
        recorder.Add("FinalReward", GetCumulativeReward());
    }
    
    // Optional: Visualize explored grid cells in Scene view
    void OnDrawGizmos(){
        if (visitedCells == null || visitedCells.Count == 0) return;
        
        // Draw visited cells
        foreach (Vector2Int cell in visitedCells){
            Vector3 worldPos = new Vector3(
                cell.x * GRID_CELL_SIZE,
                cell.y * GRID_CELL_SIZE,
                0
            );
            Gizmos.color = new Color(0, 1, 0, 0.2f);
            Gizmos.DrawCube(worldPos, Vector3.one * GRID_CELL_SIZE);
        }
        
        // Draw current cell
        if (Application.isPlaying){
            Vector2Int current = new Vector2Int(
                Mathf.RoundToInt(transform.localPosition.x / GRID_CELL_SIZE),
                Mathf.RoundToInt(transform.localPosition.y / GRID_CELL_SIZE)
            );
            Vector3 currentWorldPos = new Vector3(
                current.x * GRID_CELL_SIZE,
                current.y * GRID_CELL_SIZE,
                0
            );
            Gizmos.color = Color.yellow;
            Gizmos.DrawWireCube(currentWorldPos, Vector3.one * GRID_CELL_SIZE);
        }
    }
}