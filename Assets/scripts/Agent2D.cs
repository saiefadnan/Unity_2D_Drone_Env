using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using UnityEngine.InputSystem;
using System.Collections.Generic;

public class Agent2D : Agent
{
    [Header("Agent Configuration")]
    public bool isManualControl = false;
    public RayPerceptionSensorComponent2D raySensor;
    public Transform[] goals;
    public Transform[] obstacles;
    public AudioSource targetReached;
    public AudioSource droneHum;
    public int maxStepCount = 2000;
    float targetDistance = 0.0f;
    int obstacleCount = 0;
    Vector2 lastPos;
    float shortestPath = 0f;
    float distanceTraveled = 0f;
    int goalsReached = 0;
    int groundCollision = 0;
    int targetIndex = 0;
    int StepCnt = 0;
    Rigidbody2D rb;
    float deltaX= 0.0f;
    float targetY = 0.0f;
    float softLandingThreshold = 1.5f;  // slow = soft landing
    float hardLandingThreshold = 4f;    // fast = hard crash
    float previousDistance = Mathf.Infinity; // Track distance to target for reward shaping


    public override void Initialize(){
        rb = GetComponent<Rigidbody2D>();
    }
    Vector2 GetRandomTargetPosition(){
        Vector2 dronePos = transform.localPosition;
        float minDistance = 2f;
        if(targetDistance >=10f) minDistance = 5f;
        float distance = Random.Range(minDistance, Mathf.Max(minDistance, targetDistance));
        float forbiddenAngle = Mathf.PI / 2f; // straight up
        float angleOffset = Mathf.PI / 6f;    // 30° buffer
        float angle;
        do {
            angle = Random.Range(0f, Mathf.PI);
        } while (angle > (forbiddenAngle - angleOffset) && angle < (forbiddenAngle + angleOffset));
        float offsetX = Mathf.Cos(angle) * distance;
        float offsetY = Mathf.Sin(angle) * distance;
            // Raw target
        float targetX = dronePos.x + offsetX;
        float targetY = dronePos.y + offsetY;
        // Clamp to valid flight area
        targetX = Mathf.Clamp(targetX, -16f, 16f);
        targetY = Mathf.Clamp(targetY, -6f, 8f);

        return new Vector2(targetX, targetY);
    }
    void InitCurriculumEnv(){
        var envParams = Academy.Instance.EnvironmentParameters;
        targetDistance = envParams.GetWithDefault("target_distance", 0.0f);
        obstacleCount = (int)envParams.GetWithDefault("obstacle_count", 0f);
    }
    
    float DistanceToTarget(){
        if (!goals[targetIndex].gameObject.activeSelf) return 0f;
        return Vector2.Distance(goals[targetIndex].localPosition, transform.localPosition);
    }
    void calculateShortestPath(){
       shortestPath+=deltaX;
       int k = targetIndex;
       Dictionary<int, bool> visited = new Dictionary<int, bool>();
       visited[k] = true;
       while(visited.Count < goals.Length){
           float nearestDist = Mathf.Infinity;
           int nearestIndex = -1;
           for(int i=0; i<goals.Length; i++){
               if(!visited.ContainsKey(i) && i!= k){
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
        InitCurriculumEnv();
        RecordStats();
        rb.linearVelocity = Vector2.zero;
        rb.angularVelocity = 0f;
        targetY = transform.localPosition.y;
        shortestPath = 0f;
        lastPos = transform.localPosition;
        distanceTraveled = 0f;
        StepCnt = 0;
        groundCollision = 0;
        if (goalsReached != 0){
            goalsReached = 0;
            foreach (Transform goal in goals){
                goal.gameObject.SetActive(true);
            }
        }
        transform.localPosition = new Vector2(Random.Range(-7f, 7f), -1.93f);
        rb.SetRotation(0f);
        rb.Sleep();
        rb.WakeUp();
        for (int i = 0; i < goals.Length; i++){
            goals[i].localPosition = GetRandomTargetPosition();
        }
        int maxObs = Mathf.Min(obstacleCount, obstacles.Length); // safety check
        for (int i = 0; i < maxObs; i++){
            int randomIndex = Random.Range(i, obstacles.Length);
            Vector2 pos1 = obstacles[i].localPosition;
            Vector2 pos2 = obstacles[randomIndex].localPosition;
            obstacles[i].localPosition = new Vector2(pos2.x, pos1.y);
            obstacles[randomIndex].localPosition = new Vector2(pos1.x, pos2.y);
            obstacles[i].gameObject.SetActive(true); // make sure it's visible
        }

        // 7️⃣ Deactivate unused obstacles
        for (int i = maxObs; i < obstacles.Length; i++){
            obstacles[i].gameObject.SetActive(false);
        }

        deltaX = GetNearestDistance(transform.localPosition);
        calculateShortestPath();
        previousDistance = DistanceToTarget(); // Initialize distance tracking
    }

    public override void CollectObservations(VectorSensor sensor){
        // Agent state
        sensor.AddObservation(rb.linearVelocity/10f); // 2 values
        sensor.AddObservation(rb.angularVelocity/180f); // 1 value
        sensor.AddObservation(transform.localPosition/16f); // 2 values
        
        // Agent orientation for stability
        float normalizedRotation = transform.eulerAngles.z;
        if (normalizedRotation > 180) normalizedRotation -= 360;
        sensor.AddObservation(normalizedRotation / 180f); // 1 value, normalized to [-1,1]
        
        // Target information - CRITICAL for navigation!
        // Vector2 relativeTargetPos = (Vector2)goals[targetIndex].localPosition - (Vector2)transform.localPosition;
        // sensor.AddObservation(relativeTargetPos.normalized); // 2 values - direction to target
        // sensor.AddObservation(relativeTargetPos.magnitude / 35f); // 1 value - distance (normalized)
        
        // Total: 9 observations (was 6)
    }

    void Update()
    {
        // float speed = rb.linearVelocity.magnitude;
        // droneHum.pitch = Mathf.Lerp(1.0f, 1.5f, speed / 10f);
        // Check if agent fell off
        if (transform.localPosition.y < -6f || transform.localPosition.y > 8f || transform.localPosition.x < -16f || transform.localPosition.x > 16f){ // pick a suitable value below the floor
            AddReward(-10f); // optional: give negative reward
            EndEpisode();
        }
        if (raySensor != null){
            DrawRaySensorDebug();
        }
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        StepCnt++;
        if (StepCnt > maxStepCount){
            AddReward(-1f); // Penalty for timeout
            EndEpisode();
            return;
        }
        
        //drone control
        float forceX = actions.ContinuousActions[0]; //-1 to +1
        float forceY = actions.ContinuousActions[1]; //-1 to +1
        float torque = actions.ContinuousActions[2]; //-1 to +1
        rb.AddForce(new Vector2(forceX, forceY) * 10f, ForceMode2D.Force);
        rb.AddTorque(torque * 0.5f);


         if (isManualControl){
            // Skip rewards, penalties, and curriculum stats
            previousDistance = DistanceToTarget(); // still update distance for LSTM consistency
            lastPos = transform.localPosition;      // update lastPos for continuity
            return;
        }
        // Stability rewards
        float angleZ = transform.eulerAngles.z;
        float normalizedZRotation = angleZ > 180 ? angleZ - 360 : angleZ;
        float uprightReward = 1.0f - (Mathf.Abs(normalizedZRotation) / 180.0f);
        
        if(Mathf.Abs(normalizedZRotation) > 72f){
            AddReward(-0.02f);
            if(Mathf.Abs(normalizedZRotation) > 85f)
            {
                AddReward(-2.0f); // Increased penalty for crash
                EndEpisode();
                return;
            }
        }

        // Progress towards target - KEY ADDITION!
        float currentDistance = DistanceToTarget();
        float delta = currentDistance!=0?(previousDistance - currentDistance):0;
        AddReward(Mathf.Clamp(delta * 2f, -0.01f, 0.05f));
        previousDistance = currentDistance;
        
        // Obstacle proximity penalty
        // float minObstacleDist = GetMinObstacleDistance();
        // if (minObstacleDist < 2.0f)
        // {
        //     AddReward(-0.05f * (2.0f - minObstacleDist)); // Proportional penalty
        // }

        // Path efficiency
        float newDist = Vector2.Distance(transform.localPosition, lastPos);
        if (newDist < 0.001f){
            AddReward(-0.01f); // Increased penalty for not moving
        }
        distanceTraveled += newDist;
        lastPos = transform.localPosition;
        // Small negative reward to encourage efficiency and exploration
        AddReward(-0.001f);
        //collecting data
        var recorder = Academy.Instance.StatsRecorder;
        recorder.Add("AngleStability", uprightReward);
    }
    
    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var act = actionsOut.ContinuousActions;
        // Horizontal: left/right arrows or A/D
        float horiz = 0f;
        if (Keyboard.current.aKey.isPressed) horiz = -1f;
        else if (Keyboard.current.dKey.isPressed) horiz = 1f;
        // Vertical: up/down arrows or W/S
        float vert = 0f;
        if (Keyboard.current.wKey.isPressed) vert = 1f;
        else if (Keyboard.current.sKey.isPressed) vert = -1f;
        float torque = 0f;
        if (Keyboard.current.qKey.isPressed) torque = 1f;
        else if (Keyboard.current.eKey.isPressed) torque = -1f;
        act[0] = horiz; // X force
        act[1] = vert;  // Y force
        act[2] = torque; // torque
    }

    void OnCollisionEnter2D(Collision2D other){
        if (other.gameObject.CompareTag("Ground")){
            groundCollision++;
            float impactSpeed = other.relativeVelocity.magnitude;
            if(1 < impactSpeed && impactSpeed < softLandingThreshold){
                AddReward(1f); // Soft landing reward
                Debug.Log("Soft landing.");
            }
            else if(impactSpeed >= hardLandingThreshold){
                AddReward(-5f); // Increased hard crash penalty
                EndEpisode();
                Debug.Log("Hard crash.");
                return;
            }
            else{
                AddReward(-0.2f); // Moderate penalty
                Debug.Log("Moderate landing.");
            }
        }
        
        if (other.gameObject.CompareTag("Obstacle")){
            AddReward(-5f); // Strong penalty for obstacle collision
            EndEpisode(); // End episode on obstacle crash
            Debug.Log("Crashed into obstacle!");
            return;
        }
    }

    void OnTriggerEnter2D(Collider2D other) {
        if (other.CompareTag("Victim")){
            goalsReached++;
            
            // Higher rewards for finding victims
            if (System.Array.IndexOf(goals, other.transform) == targetIndex)
            {
                AddReward(5.0f); // Increased reward for correct target
                Debug.Log("Found target victim!");
            }
            else
            {
                AddReward(2.0f); // Increased reward for any victim
                Debug.Log("Found victim!");
            }
            other.gameObject.SetActive(false);
        }
        
        if (goalsReached == goals.Length){
            float efficiency = shortestPath / Mathf.Max(distanceTraveled, 0.001f);
            var recorder = Academy.Instance.StatsRecorder;
            recorder.Add("Efficiency", efficiency);
            
            // Big success bonus!
            AddReward(efficiency * 5f + 10f); // Efficiency bonus + completion bonus
            Debug.Log("All victims found. Mission complete!");
            EndEpisode();
        }
        else{
            deltaX = GetNearestDistance(transform.localPosition);
            previousDistance = DistanceToTarget(); // Update for new target
        }
    }
    void DrawRaySensorDebug(){
        if (raySensor == null) return;

        float rayLength = raySensor.RayLength;
        int raysPerSide = raySensor.RaysPerDirection;
        float maxAngle = raySensor.MaxRayDegrees;

        int totalRays = raysPerSide * 2 + 1;

        for (int i = 0; i < totalRays; i++){
            // Correct angle calculation: symmetric around forward
            float angle = (i - raysPerSide) * (maxAngle / raysPerSide);

            // Local to world direction
            Vector3 dir = Quaternion.Euler(0f, 0f, angle) * raySensor.transform.up;
            Vector3 startPos = raySensor.transform.position;

            // Use the same layer mask as the ML-Agents sensor
            LayerMask layerMask = raySensor.GetComponent<RayPerceptionSensorComponent2D>().RayLayerMask;
            RaycastHit2D hit = Physics2D.Raycast(startPos, dir, rayLength, layerMask);

            Color rayColor = Color.red;
            
            if (hit.collider != null && hit.collider.gameObject != gameObject)
            {
                // Check if the tag is in the detectable tags list
                string[] detectableTags = raySensor.DetectableTags.ToArray();
                
                if (System.Array.IndexOf(detectableTags, hit.collider.tag) >= 0)
                {
                    if (hit.collider.CompareTag("Victim")){
                        rayColor = Color.green;
                    }
                    else if (hit.collider.CompareTag("Ground") || hit.collider.CompareTag("Obstacle")) {
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
        // Make a label in the top-left corner
        GUIStyle style = new GUIStyle();
        style.fontSize = 20;
        style.normal.textColor = Color.white;
        
        // Display the cumulative reward
        GUI.Label(new Rect(15, 25, 200, 30), 
                "Reward: " + GetCumulativeReward().ToString("F2"), style);
        GUI.Label(new Rect(15, 60, 300, 30), $"TargetDistance: {targetDistance}, Obstacles: {obstacleCount}");
    }

    void RecordStats(){
        var recorder = Academy.Instance.StatsRecorder;
        recorder.Add("EpisodeLength", StepCnt);
        recorder.Add("TargetsFound", goalsReached);
        recorder.Add("GroundCollision", groundCollision);
        recorder.Add("PathEfficiency", shortestPath / Mathf.Max(distanceTraveled, 0.001f));
        recorder.Add("FinalReward", GetCumulativeReward());
    }

}
