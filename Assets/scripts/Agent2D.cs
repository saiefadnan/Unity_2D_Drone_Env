using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using UnityEngine.InputSystem;
using System.Collections.Generic;

public class Agent2D : Agent
{
    [Header("Agent Configuration")]
    public RayPerceptionSensorComponent2D raySensor;
    public Transform[] goals;
    public Transform[] obstacles;
    public AudioSource targetReached;
    public AudioSource droneHum;
    Vector2 lastPos;
    float shortestPath = 0.0f;
    float distanceTraveled = 0.0f;
    int goalsReached = 0;
    bool groundCollision = false;
    int targetIndex = 0;
    int StepCnt = 0;
    Rigidbody2D rb;
    float deltaX= 0.0f;
    float targetY = 0.0f;
    float softLandingThreshold = 1.5f;  // slow = soft landing
    float hardLandingThreshold = 4f;    // fast = hard crash
    float previousDistance = Mathf.Infinity; // Track distance to target for reward shaping


    public override void Initialize()
    {
        rb = GetComponent<Rigidbody2D>();
    }
    
    float DistanceToTarget(){
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
    public override void OnEpisodeBegin()
    {
        
        if (StepCnt > 0)
        {
            var recorder = Academy.Instance.StatsRecorder;
            recorder.Add("EpisodeLength", StepCnt);
            recorder.Add("TargetsFound", goalsReached);
            recorder.Add("PathEfficiency", shortestPath / Mathf.Max(distanceTraveled, 0.001f));
        }
        rb.linearVelocity = Vector2.zero;
        rb.angularVelocity = 0f;
        targetY = transform.localPosition.y;
        shortestPath = 0.0f;
        lastPos = transform.localPosition;
        distanceTraveled = 0.0f;
        StepCnt = 0;
        groundCollision = false;
        if (goalsReached != 0)
        {
            goalsReached = 0;
            foreach (Transform goal in goals)
            {
                goal.gameObject.SetActive(true);
            }
        }
        transform.localPosition = new Vector2(Random.Range(-7f, 7f), -1.93f);
        rb.SetRotation(0f);
        rb.Sleep();
        rb.WakeUp();
        for (int i = 0; i < goals.Length; i++){
            goals[i].localPosition = new Vector2(Random.Range(-15f, 15f), Random.Range(-3f, 2f));
        }
        for (int i = 0; i < obstacles.Length; i++){
            int randomIndex = Random.Range(i, obstacles.Length);
            Vector2 pos1 = obstacles[i].localPosition;
            Vector2 pos2 = obstacles[randomIndex].localPosition;
            obstacles[i].localPosition = new Vector2(pos2.x, pos1.y);
            obstacles[randomIndex].localPosition = new Vector2(pos1.x, pos2.y);
        }
        deltaX = GetNearestDistance(transform.localPosition);
        calculateShortestPath();
        previousDistance = DistanceToTarget(); // Initialize distance tracking
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        // Agent state
        sensor.AddObservation(rb.linearVelocity); // 2 values
        sensor.AddObservation(rb.angularVelocity); // 1 value
        sensor.AddObservation(transform.localPosition); // 2 values
        
        // Agent orientation for stability
        float normalizedRotation = transform.eulerAngles.z;
        if (normalizedRotation > 180) normalizedRotation -= 360;
        sensor.AddObservation(normalizedRotation / 180f); // 1 value, normalized to [-1,1]
        
        // Target information - CRITICAL for navigation!
        Vector2 relativeTargetPos = (Vector2)goals[targetIndex].localPosition - (Vector2)transform.localPosition;
        sensor.AddObservation(relativeTargetPos.normalized); // 2 values - direction to target
        sensor.AddObservation(relativeTargetPos.magnitude / 35f); // 1 value - distance (normalized)
        
        // Total: 9 observations (was 6)
    }

    void Update()
    {
        // float speed = rb.linearVelocity.magnitude;
        // droneHum.pitch = Mathf.Lerp(1.0f, 1.5f, speed / 10f);
        // Check if agent fell off
        if (transform.localPosition.y < -6f || transform.localPosition.y > 8f || transform.localPosition.x < -16f || transform.localPosition.x > 16f){ // pick a suitable value below the floor
            AddReward(-50f); // optional: give negative reward
            EndEpisode();
        }
         if (raySensor != null){
            DrawRaySensorDebug();
        }
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        StepCnt++;
        if (StepCnt > 10000){
            AddReward(-1f); // Penalty for timeout
            EndEpisode();
            return;
        }
        
        float forceX = actions.ContinuousActions[0]; //-1 to +1
        float forceY = actions.ContinuousActions[1]; //-1 to +1
        float torque = actions.ContinuousActions[2]; //-1 to +1
        
        rb.AddForce(new Vector2(forceX, forceY) * 10f, ForceMode2D.Force);
        rb.AddTorque(torque * 0.5f);

        // Stability rewards
        float angleZ = transform.eulerAngles.z;
        float normalizedZRotation = angleZ > 180 ? angleZ - 360 : angleZ;
        float uprightReward = 1.0f - (Mathf.Abs(normalizedZRotation) / 180.0f);
        
        if(Mathf.Abs(normalizedZRotation) > 72f)
        {
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
        float approachReward = (previousDistance - currentDistance) * 0.5f;
        AddReward(approachReward); // Positive if getting closer, negative if moving away
        previousDistance = currentDistance;
        
        // Obstacle proximity penalty
        float minObstacleDist = GetMinObstacleDistance();
        if (minObstacleDist < 2.0f)
        {
            AddReward(-0.05f * (2.0f - minObstacleDist)); // Proportional penalty
        }

        // Path efficiency
        float newDist = Vector2.Distance(transform.localPosition, lastPos);
        if (newDist < 0.001f){
            AddReward(-0.1f); // Increased penalty for not moving
        }
        distanceTraveled += newDist;
        lastPos = transform.localPosition;
        // Small negative reward to encourage efficiency and exploration
        AddReward(-0.001f);
        //collecting data
        RecordStats(uprightReward);
    }
    
    float GetMinObstacleDistance()
    {
        float minDist = Mathf.Infinity;
        Collider2D[] nearbyObjects = Physics2D.OverlapCircleAll(transform.position, 3f);
        
        foreach(var col in nearbyObjects)
        {
            if (col != null && (col.CompareTag("Obstacle") || col.CompareTag("Ground")))
            {
                float dist = Vector2.Distance(transform.position, col.transform.position);
                if (dist < minDist && dist > 0.1f) // Ignore very close/self collisions
                {
                    minDist = dist;
                }
            }
        }
        return minDist;
    }

    void RecordStats(float uprightReward)
    {
        var recorder = Academy.Instance.StatsRecorder;
        recorder.Add("AngleStability", uprightReward);
        recorder.Add("GroundCollision", groundCollision ? 1f : 0f);
        recorder.Add("Reward", GetCumulativeReward());
        recorder.Add("TargetsFound", goalsReached); // Simple metric
        
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
            groundCollision = true;
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
        
        if (goalsReached == 5){
            float efficiency = shortestPath / Mathf.Max(distanceTraveled, 0.001f);
            var recorder = Academy.Instance.StatsRecorder;
            recorder.Add("Efficiency", efficiency);
            
            // Big success bonus!
            AddReward(efficiency * 10.0f + 20.0f); // Efficiency bonus + completion bonus
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
                        //if (Time.frameCount % 60 == 0) // Reduce console spam
                            //Debug.Log($"Ray {i} hit Victim: {hit.collider.name} at distance: {hit.distance}");
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
    }

}

