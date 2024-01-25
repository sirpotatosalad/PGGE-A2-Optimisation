using System.Collections;
using System.Collections.Generic;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using UnityEngine;
using UnityEngine.EventSystems;

public class FlockBehaviour : MonoBehaviour
{
  List<Obstacle> mObstacles = new List<Obstacle>();

  [SerializeField]
  GameObject[] Obstacles;

  [SerializeField]
  BoxCollider2D Bounds;

  public float TickDuration = 1.0f;
  public float TickDurationSeparationEnemy = 0.1f;
  public float TickDurationRandom = 1.0f;

  public int BoidIncr = 100;
  public bool useFlocking = false;
  public int BatchSize = 100;

  public List<Flock> flocks = new List<Flock>();
  void Reset()
  {
    flocks = new List<Flock>()
    {
      new Flock()
    };
  }

  void Start()
  {
    // Randomize obstacles placement.
    for(int i = 0; i < Obstacles.Length; ++i)
    {
      float x = Random.Range(Bounds.bounds.min.x, Bounds.bounds.max.x);
      float y = Random.Range(Bounds.bounds.min.y, Bounds.bounds.max.y);
      Obstacles[i].transform.position = new Vector3(x, y, 0.0f);
      Obstacle obs = Obstacles[i].AddComponent<Obstacle>();
      Autonomous autono = Obstacles[i].AddComponent<Autonomous>();
      autono.MaxSpeed = 1.0f;
      obs.mCollider = Obstacles[i].GetComponent<CircleCollider2D>();
      mObstacles.Add(obs);
    }

    foreach (Flock flock in flocks)
    {
      CreateFlock(flock);
    }

    StartCoroutine(Coroutine_Flocking());

    StartCoroutine(Coroutine_Random());
    StartCoroutine(Coroutine_AvoidObstacles());
    StartCoroutine(Coroutine_SeparationWithEnemies());
    StartCoroutine(Coroutine_Random_Motion_Obstacles());
  }

  void CreateFlock(Flock flock)
  {
    for(int i = 0; i < flock.numBoids; ++i)
    {
      float x = Random.Range(Bounds.bounds.min.x, Bounds.bounds.max.x);
      float y = Random.Range(Bounds.bounds.min.y, Bounds.bounds.max.y);

      AddBoid(x, y, flock);
    }
  }

  void Update()
  {
    HandleInputs();
    Rule_CrossBorder();
    Rule_CrossBorder_Obstacles();
  }

  void HandleInputs()
  {
    if (EventSystem.current.IsPointerOverGameObject() ||
       enabled == false)
    {
      return;
    }

    if (Input.GetKeyDown(KeyCode.Space))
    {
      AddBoids(BoidIncr);
    }
  }

  void AddBoids(int count)
  {
    for(int i = 0; i < count; ++i)
    {
      float x = Random.Range(Bounds.bounds.min.x, Bounds.bounds.max.x);
      float y = Random.Range(Bounds.bounds.min.y, Bounds.bounds.max.y);

      AddBoid(x, y, flocks[0]);
    }
    flocks[0].numBoids += count;
  }

  void AddBoid(float x, float y, Flock flock)
  {
    GameObject obj = Instantiate(flock.PrefabBoid);
    obj.name = "Boid_" + flock.name + "_" + flock.mAutonomous.Count;
    obj.transform.position = new Vector3(x, y, 0.0f);
    Autonomous boid = obj.GetComponent<Autonomous>();
    flock.mAutonomous.Add(boid);
    boid.MaxSpeed = flock.maxSpeed;
    boid.RotationSpeed = flock.maxRotationSpeed;
  }

  static float Distance(Autonomous a1, Autonomous a2)
  {
    return (a1.transform.position - a2.transform.position).sqrMagnitude;
  }

    //void Execute(Flock flock, int i)
    //{
    //  Vector3 flockDir = Vector3.zero;
    //  Vector3 separationDir = Vector3.zero;
    //  Vector3 cohesionDir = Vector3.zero;

    //  float speed = 0.0f;
    //  float separationSpeed = 0.0f;

    //  int count = 0;
    //  int separationCount = 0;
    //  Vector3 steerPos = Vector3.zero;

    //  Autonomous curr = flock.mAutonomous[i];
    //  for (int j = 0; j < flock.numBoids; ++j)
    //  {
    //    Autonomous other = flock.mAutonomous[j];
    //    float dist = (curr.transform.position - other.transform.position).magnitude;
    //    if (i != j && dist < flock.visibility)
    //    {
    //      speed += other.Speed;
    //      flockDir += other.TargetDirection;
    //      steerPos += other.transform.position;
    //      count++;
    //    }
    //    if (i != j)
    //    {
    //      if (dist < flock.separationDistance)
    //      {
    //        Vector3 targetDirection = (
    //          curr.transform.position -
    //          other.transform.position).normalized;

    //        separationDir += targetDirection;
    //        separationSpeed += dist * flock.weightSeparation;
    //      }
    //    }
    //  }
    //  if (count > 0)
    //  {
    //    speed = speed / count;
    //    flockDir = flockDir / count;
    //    flockDir.Normalize();

    //    steerPos = steerPos / count;
    //  }

    //  if (separationCount > 0)
    //  {
    //    separationSpeed = separationSpeed / count;
    //    separationDir = separationDir / separationSpeed;
    //    separationDir.Normalize();
    //  }

    //  curr.TargetDirection =
    //    flockDir * speed * (flock.useAlignmentRule ? flock.weightAlignment : 0.0f) +
    //    separationDir * separationSpeed * (flock.useSeparationRule ? flock.weightSeparation : 0.0f) +
    //    (steerPos - curr.transform.position) * (flock.useCohesionRule ? flock.weightCohesion : 0.0f);
    //}

    void Execute(Flock flock, int i)
    {
        Vector3 flockDir = Vector3.zero;
        Vector3 separationDir = Vector3.zero;
        Vector3 steerPos = Vector3.zero;

        float speed = 0.0f;
        float separationSpeed = 0.0f;

        int count = 0;

        Autonomous curr = flock.mAutonomous[i];
        Vector3 currPosition = curr.transform.position;

        float visibilitySquared = flock.visibility * flock.visibility;
        float separationDistanceSquared = flock.separationDistance * flock.separationDistance;


        for (int j = 0; j < flock.numBoids; ++j)
        {
            if (i == j) continue; // Skip self

            Autonomous other = flock.mAutonomous[j];
            Vector3 otherPosition = other.transform.position;

            Vector3 relativePos = currPosition - otherPosition;
            float distSquared = relativePos.sqrMagnitude;

            if (distSquared < visibilitySquared)
            {
                speed += other.Speed;
                flockDir += other.TargetDirection;
                steerPos += otherPosition;
                count++;

                if (distSquared < separationDistanceSquared)
                {
                    separationDir += relativePos.normalized;
                    separationSpeed += distSquared * flock.weightSeparation;
                }
            }
        }

        if (count > 0)
        {
            speed /= count;
            flockDir /= count;
            flockDir.Normalize();

            steerPos /= count;
        }

        if (count > 1)
        {
            separationSpeed /= count - 1;
            separationDir.Normalize();
        }

        curr.TargetDirection =
            flockDir * speed * (flock.useAlignmentRule ? flock.weightAlignment : 0.0f) +
            separationDir * separationSpeed * (flock.useSeparationRule ? flock.weightSeparation : 0.0f) +
            (steerPos - currPosition) * (flock.useCohesionRule ? flock.weightCohesion : 0.0f);
    }


    //IEnumerator Coroutine_Flocking()
    //{
    //    WaitForSeconds wait = new WaitForSeconds(TickDuration);
    //    float maxFrameTime = 0.016f;

    //    while (useFlocking)
    //    {
    //        foreach (Flock flock in flocks)
    //        {
    //            List<Autonomous> autonomousList = flock.mAutonomous;
    //            int index = 0;

    //            while (index < autonomousList.Count)
    //            {
    //                int endIndex = Mathf.Min(index + BatchSize, autonomousList.Count);

    //                for (int j = index; j < endIndex; j++)
    //                {
    //                    Execute(flock, j);
    //                }

    //                index = endIndex;

    //                float frameTime = Time.deltaTime;

    //                if (frameTime > maxFrameTime)
    //                {
    //                    // If frame time exceeds the maximum target, yield and resume in the next frame
    //                    yield return null;
    //                    break;
    //                }
    //            }
    //        }

    //        yield return wait;
    //    }
    //}

    IEnumerator Coroutine_Flocking()
    {
        WaitForSeconds wait = new WaitForSeconds(TickDuration);

        while (useFlocking)
        {
            foreach (Flock flock in flocks)
            {
                List<Autonomous> autonomousList = flock.mAutonomous;

                // Allocate NativeArrays to pass data to the job
                NativeArray<Vector3> positions = new NativeArray<Vector3>(autonomousList.Count, Allocator.TempJob);
                NativeArray<Vector3> directions = new NativeArray<Vector3>(autonomousList.Count, Allocator.TempJob);
                NativeArray<float> speeds = new NativeArray<float>(autonomousList.Count, Allocator.TempJob);
                NativeArray<Vector3> modifiedPositions = new NativeArray<Vector3>(autonomousList.Count, Allocator.TempJob);
                NativeArray<Vector3> modifiedDirections = new NativeArray<Vector3>(autonomousList.Count, Allocator.TempJob);
                NativeArray<float> modifiedSpeeds = new NativeArray<float>(autonomousList.Count, Allocator.TempJob);

                // Populate NativeArrays with current data
                for (int i = 0; i < autonomousList.Count; ++i)
                {
                    positions[i] = autonomousList[i].transform.position;
                    directions[i] = autonomousList[i].TargetDirection;
                    speeds[i] = autonomousList[i].Speed;
                }

                // Create a FlockingJob and set its parameters
                FlockingJob flockingJob = new FlockingJob
                {
                    VisibilitySquared = flock.visibility * flock.visibility,
                    SeparationDistanceSquared = flock.separationDistance * flock.separationDistance,
                    WeightAlignment = flock.weightAlignment,
                    WeightSeparation = flock.weightSeparation,
                    WeightCohesion = flock.weightCohesion,
                    AutonomousPositions = positions,
                    AutonomousDirections = directions,
                    AutonomousSpeeds = speeds,
                    ModifiedPositions = modifiedPositions,
                    ModifiedDirections = modifiedDirections,
                    ModifiedSpeeds = modifiedSpeeds
                };

                // Schedule the job
                JobHandle jobHandle = flockingJob.Schedule(autonomousList.Count, BatchSize);

                // Wait for the job to complete
                jobHandle.Complete();

                // Update the modified data back to the Autonomous instances
                for (int i = 0; i < autonomousList.Count; ++i)
                {
                    autonomousList[i].transform.position = modifiedPositions[i];
                    autonomousList[i].TargetDirection = modifiedDirections[i];
                    autonomousList[i].SetSpeed(modifiedSpeeds[i]);
                }

                // Dispose of NativeArrays
                positions.Dispose();
                directions.Dispose();
                speeds.Dispose();
                modifiedPositions.Dispose();
                modifiedDirections.Dispose();
                modifiedSpeeds.Dispose();
            }

            yield return wait;
        }
    }


    void SeparationWithEnemies_Internal(
    List<Autonomous> boids, 
    List<Autonomous> enemies, 
    float sepDist, 
    float sepWeight)
  {
    for(int i = 0; i < boids.Count; ++i)
    {
      for (int j = 0; j < enemies.Count; ++j)
      {
        float dist = (
          enemies[j].transform.position -
          boids[i].transform.position).magnitude;
        if (dist < sepDist)
        {
          Vector3 targetDirection = (
            boids[i].transform.position -
            enemies[j].transform.position).normalized;

          boids[i].TargetDirection += targetDirection;
          boids[i].TargetDirection.Normalize();

          boids[i].TargetSpeed += dist * sepWeight;
          boids[i].TargetSpeed /= 2.0f;
        }
      }
    }
  }

    //IEnumerator Coroutine_SeparationWithEnemies()
    //{
    //    while (true)
    //    {
    //        foreach (Flock flock in flocks)
    //        {
    //            if (!flock.useFleeOnSightEnemyRule || flock.isPredator) continue;

    //            foreach (Flock enemies in flocks)
    //            {
    //                if (!enemies.isPredator) continue;

    //                SeparationWithEnemies_Internal(
    //                  flock.mAutonomous,
    //                  enemies.mAutonomous,
    //                  flock.enemySeparationDistance,
    //                  flock.weightFleeOnSightEnemy);
    //            }
    //            //yield return null;
    //        }
    //        yield return null;
    //    }
    //}

    IEnumerator Coroutine_SeparationWithEnemies()
    {
        WaitForSeconds wait = new WaitForSeconds(TickDuration); // Use your desired TickDuration

        while (true)
        {
            foreach (Flock flock in flocks)
            {
                if (!flock.useFleeOnSightEnemyRule || flock.isPredator) continue;

                foreach (Flock enemies in flocks)
                {
                    if (!enemies.isPredator) continue;

                    List<Autonomous> boids = flock.mAutonomous;
                    List<Autonomous> enemyBoids = enemies.mAutonomous;

                    // Allocate NativeArrays to pass data to the job
                    NativeArray<Vector3> boidPositions = new NativeArray<Vector3>(boids.Count, Allocator.TempJob);
                    NativeArray<Vector3> enemyPositions = new NativeArray<Vector3>(enemyBoids.Count, Allocator.TempJob);
                    NativeArray<Vector3> modifiedDirections = new NativeArray<Vector3>(boids.Count, Allocator.TempJob);
                    NativeArray<float> modifiedSpeeds = new NativeArray<float>(boids.Count, Allocator.TempJob);

                    // Populate NativeArrays with current data
                    for (int k = 0; k < boids.Count; ++k)
                    {
                        boidPositions[k] = boids[k].transform.position;
                        modifiedDirections[k] = boids[k].TargetDirection;
                        modifiedSpeeds[k] = boids[k].TargetSpeed;
                    }

                    // Populate NativeArray with enemy positions
                    for (int l = 0; l < enemyBoids.Count; ++l)
                    {
                        enemyPositions[l] = enemyBoids[l].transform.position;
                    }

                    // Create a SeparationWithEnemiesJob and set its parameters
                    SeparationWithEnemiesJob separationJob = new SeparationWithEnemiesJob
                    {
                        SeparationDistance = flock.enemySeparationDistance,
                        SeparationWeight = flock.weightFleeOnSightEnemy,
                        BoidPositions = boidPositions,
                        EnemyPositions = enemyPositions,
                        ModifiedDirections = modifiedDirections,
                        ModifiedSpeeds = modifiedSpeeds
                    };

                    // Schedule the job
                    JobHandle jobHandle = separationJob.Schedule(boids.Count, BatchSize);

                    // Wait for the job to complete
                    jobHandle.Complete();

                    // Update the modified data back to the Autonomous instances
                    for (int m = 0; m < boids.Count; ++m)
                    {
                        boids[m].TargetDirection = modifiedDirections[m];
                        boids[m].TargetSpeed = modifiedSpeeds[m];
                    }

                    // Dispose of NativeArrays
                    boidPositions.Dispose();
                    enemyPositions.Dispose();
                    modifiedDirections.Dispose();
                    modifiedSpeeds.Dispose();
                }
            }
            yield return wait;
        }
    }



    //IEnumerator Coroutine_AvoidObstacles()
    //{
    //    while (true)
    //    {
    //        foreach (Flock flock in flocks)
    //        {
    //            if (flock.useAvoidObstaclesRule)
    //            {
    //                List<Autonomous> autonomousList = flock.mAutonomous;
    //                for (int i = 0; i < autonomousList.Count; ++i)
    //                {
    //                    for (int j = 0; j < mObstacles.Count; ++j)
    //                    {
    //                        float dist = (
    //                          mObstacles[j].transform.position -
    //                          autonomousList[i].transform.position).magnitude;
    //                        if (dist < mObstacles[j].AvoidanceRadius)
    //                        {
    //                            Vector3 targetDirection = (
    //                              autonomousList[i].transform.position -
    //                              mObstacles[j].transform.position).normalized;

    //                            autonomousList[i].TargetDirection += targetDirection * flock.weightAvoidObstacles;
    //                            autonomousList[i].TargetDirection.Normalize();
    //                        }
    //                    }
    //                }
    //            }
    //            //yield return null;
    //        }
    //        yield return null;
    //    }
    //}

    IEnumerator Coroutine_AvoidObstacles()
    {
        WaitForSeconds wait = new WaitForSeconds(TickDuration); // Use your desired TickDuration

        while (true)
        {
            foreach (Flock flock in flocks)
            {
                if (flock.useAvoidObstaclesRule)
                {
                    List<Autonomous> autonomousList = flock.mAutonomous;

                    // Allocate NativeArrays to pass data to the job
                    NativeArray<Vector3> autonomousPositions = new NativeArray<Vector3>(autonomousList.Count, Allocator.TempJob);
                    NativeArray<Vector3> obstaclePositions = new NativeArray<Vector3>(mObstacles.Count, Allocator.TempJob);
                    NativeArray<Vector3> modifiedDirections = new NativeArray<Vector3>(autonomousList.Count, Allocator.TempJob);

                    // Populate NativeArrays with current data
                    for (int i = 0; i < autonomousList.Count; ++i)
                    {
                        autonomousPositions[i] = autonomousList[i].transform.position;
                        modifiedDirections[i] = autonomousList[i].TargetDirection;
                    }

                    // Populate NativeArray with obstacle positions
                    for (int j = 0; j < mObstacles.Count; ++j)
                    {
                        obstaclePositions[j] = mObstacles[j].transform.position;
                    }

                    // Create an AvoidObstaclesJob and set its parameters
                    AvoidObstaclesJob avoidObstaclesJob = new AvoidObstaclesJob
                    {
                        AvoidanceRadius = flock.weightAvoidObstacles,
                        AutonomousPositions = autonomousPositions,
                        ObstaclePositions = obstaclePositions,
                        ModifiedDirections = modifiedDirections
                    };

                    // Schedule the job
                    JobHandle jobHandle = avoidObstaclesJob.Schedule(autonomousList.Count, BatchSize);

                    // Wait for the job to complete
                    jobHandle.Complete();

                    // Update the modified data back to the Autonomous instances
                    for (int i = 0; i < autonomousList.Count; ++i)
                    {
                        autonomousList[i].TargetDirection = modifiedDirections[i];
                    }

                    // Dispose of NativeArrays
                    autonomousPositions.Dispose();
                    obstaclePositions.Dispose();
                    modifiedDirections.Dispose();
                }
            }
            yield return wait;
        }
    }


    IEnumerator Coroutine_Random_Motion_Obstacles()
  {
    while (true)
    {
      for (int i = 0; i < Obstacles.Length; ++i)
      {
        Autonomous autono = Obstacles[i].GetComponent<Autonomous>();
        float rand = Random.Range(0.0f, 1.0f);
        autono.TargetDirection.Normalize();
        float angle = Mathf.Atan2(autono.TargetDirection.y, autono.TargetDirection.x);

        if (rand > 0.5f)
        {
          angle += Mathf.Deg2Rad * 45.0f;
        }
        else
        {
          angle -= Mathf.Deg2Rad * 45.0f;
        }
        Vector3 dir = Vector3.zero;
        dir.x = Mathf.Cos(angle);
        dir.y = Mathf.Sin(angle);

        autono.TargetDirection += dir * 0.1f;
        autono.TargetDirection.Normalize();
        //Debug.Log(autonomousList[i].TargetDirection);

        float speed = Random.Range(1.0f, autono.MaxSpeed);
        autono.TargetSpeed += speed;
        autono.TargetSpeed /= 2.0f;
      }
      yield return new WaitForSeconds(2.0f);
    }
  }
  IEnumerator Coroutine_Random()
  {
    while (true)
    {
      foreach (Flock flock in flocks)
      {
        if (flock.useRandomRule)
        {
          List<Autonomous> autonomousList = flock.mAutonomous;
          for (int i = 0; i < autonomousList.Count; ++i)
          {
            float rand = Random.Range(0.0f, 1.0f);
            autonomousList[i].TargetDirection.Normalize();
            float angle = Mathf.Atan2(autonomousList[i].TargetDirection.y, autonomousList[i].TargetDirection.x);

            if (rand > 0.5f)
            {
              angle += Mathf.Deg2Rad * 45.0f;
            }
            else
            {
              angle -= Mathf.Deg2Rad * 45.0f;
            }
            Vector3 dir = Vector3.zero;
            dir.x = Mathf.Cos(angle);
            dir.y = Mathf.Sin(angle);

            autonomousList[i].TargetDirection += dir * flock.weightRandom;
            autonomousList[i].TargetDirection.Normalize();
            //Debug.Log(autonomousList[i].TargetDirection);

            float speed = Random.Range(1.0f, autonomousList[i].MaxSpeed);
            autonomousList[i].TargetSpeed += speed * flock.weightSeparation;
            autonomousList[i].TargetSpeed /= 2.0f;
          }
        }
        //yield return null;
      }
      yield return new WaitForSeconds(TickDurationRandom);
    }
  }
  void Rule_CrossBorder_Obstacles()
  {
    for (int i = 0; i < Obstacles.Length; ++i)
    {
      Autonomous autono = Obstacles[i].GetComponent<Autonomous>();
      Vector3 pos = autono.transform.position;
      if (autono.transform.position.x > Bounds.bounds.max.x)
      {
        pos.x = Bounds.bounds.min.x;
      }
      if (autono.transform.position.x < Bounds.bounds.min.x)
      {
        pos.x = Bounds.bounds.max.x;
      }
      if (autono.transform.position.y > Bounds.bounds.max.y)
      {
        pos.y = Bounds.bounds.min.y;
      }
      if (autono.transform.position.y < Bounds.bounds.min.y)
      {
        pos.y = Bounds.bounds.max.y;
      }
      autono.transform.position = pos;
    }

    //for (int i = 0; i < Obstacles.Length; ++i)
    //{
    //  Autonomous autono = Obstacles[i].GetComponent<Autonomous>();
    //  Vector3 pos = autono.transform.position;
    //  if (autono.transform.position.x + 5.0f > Bounds.bounds.max.x)
    //  {
    //    autono.TargetDirection.x = -1.0f;
    //  }
    //  if (autono.transform.position.x - 5.0f < Bounds.bounds.min.x)
    //  {
    //    autono.TargetDirection.x = 1.0f;
    //  }
    //  if (autono.transform.position.y + 5.0f > Bounds.bounds.max.y)
    //  {
    //    autono.TargetDirection.y = -1.0f;
    //  }
    //  if (autono.transform.position.y - 5.0f < Bounds.bounds.min.y)
    //  {
    //    autono.TargetDirection.y = 1.0f;
    //  }
    //  autono.TargetDirection.Normalize();
    //}
  }

  void Rule_CrossBorder()
  {
    foreach (Flock flock in flocks)
    {
      List<Autonomous> autonomousList = flock.mAutonomous;
      if (flock.bounceWall)
      {
        for (int i = 0; i < autonomousList.Count; ++i)
        {
            if (autonomousList[i] != null)
            {
                Vector3 pos = autonomousList[i].transform.position;
                if (autonomousList[i].transform.position.x + 5.0f > Bounds.bounds.max.x)
                {
                    autonomousList[i].TargetDirection.x = -1.0f;
                }
                if (autonomousList[i].transform.position.x - 5.0f < Bounds.bounds.min.x)
                {
                    autonomousList[i].TargetDirection.x = 1.0f;
                }
                if (autonomousList[i].transform.position.y + 5.0f > Bounds.bounds.max.y)
                {
                    autonomousList[i].TargetDirection.y = -1.0f;
                }
                if (autonomousList[i].transform.position.y - 5.0f < Bounds.bounds.min.y)
                {
                    autonomousList[i].TargetDirection.y = 1.0f;
                }
                autonomousList[i].TargetDirection.Normalize();

            }
          
        }
      }
      else
      {
        for (int i = 0; i < autonomousList.Count; ++i)
        {
          if (autonomousList[i] != null)
            {
                Vector3 pos = autonomousList[i].transform.position;
                if (autonomousList[i].transform.position.x > Bounds.bounds.max.x)
                {
                    pos.x = Bounds.bounds.min.x;
                }
                if (autonomousList[i].transform.position.x < Bounds.bounds.min.x)
                {
                    pos.x = Bounds.bounds.max.x;
                }
                if (autonomousList[i].transform.position.y > Bounds.bounds.max.y)
                {
                    pos.y = Bounds.bounds.min.y;
                }
                if (autonomousList[i].transform.position.y < Bounds.bounds.min.y)
                {
                    pos.y = Bounds.bounds.max.y;
                }
                autonomousList[i].transform.position = pos;
            }
          
        }
      }
    }
  }

    [BurstCompile]
    public struct FlockingJob : IJobParallelFor
    {
        // Parameters for the flocking behavior
        public float VisibilitySquared;
        public float SeparationDistanceSquared;
        public float WeightAlignment;
        public float WeightSeparation;
        public float WeightCohesion;

        [ReadOnly] public NativeArray<Vector3> AutonomousPositions;
        [ReadOnly] public NativeArray<Vector3> AutonomousDirections;
        [ReadOnly] public NativeArray<float> AutonomousSpeeds;

        public NativeArray<Vector3> ModifiedPositions;
        public NativeArray<Vector3> ModifiedDirections;
        public NativeArray<float> ModifiedSpeeds;

        // You may need to add more data members based on the specifics of your logic

        public void Execute(int i)
        {
            Vector3 currPosition = AutonomousPositions[i];
            Vector3 currDirection = AutonomousDirections[i];
            float currSpeed = AutonomousSpeeds[i];

            Vector3 flockDir = Vector3.zero;
            Vector3 separationDir = Vector3.zero;
            Vector3 steerPos = Vector3.zero;

            float speed = 0.0f;
            float separationSpeed = 0.0f;

            int count = 0;

            for (int j = 0; j < AutonomousPositions.Length; ++j)
            {
                if (i == j)
                    continue; // Skip self

                Vector3 otherPosition = AutonomousPositions[j];
                Vector3 relativePos = currPosition - otherPosition;
                float distSquared = relativePos.sqrMagnitude;

                if (distSquared < VisibilitySquared)
                {
                    speed += AutonomousSpeeds[j];
                    flockDir += AutonomousDirections[j];
                    steerPos += otherPosition;
                    count++;

                    if (distSquared < SeparationDistanceSquared)
                    {
                        separationDir += relativePos.normalized;
                        separationSpeed += distSquared * WeightSeparation;
                    }
                }
            }

            if (count > 0)
            {
                speed /= count;
                flockDir /= count;
                flockDir.Normalize();
                steerPos /= count;
            }

            if (count > 1)
            {
                separationSpeed /= count - 1;
                separationDir.Normalize();
            }

            // Apply flocking rules
            currDirection =
                flockDir * speed * (WeightAlignment > 0 ? WeightAlignment : 0.0f) +
                separationDir * separationSpeed * (WeightSeparation > 0 ? WeightSeparation : 0.0f) +
                (steerPos - currPosition) * (WeightCohesion > 0 ? WeightCohesion : 0.0f);

            // Update the modified data back to the arrays
            ModifiedPositions[i] = currPosition;
            ModifiedDirections[i] = currDirection.normalized; // Normalize the direction
            ModifiedSpeeds[i] = speed;
        }
    }

    [BurstCompile]
    public struct AvoidObstaclesJob : IJobParallelFor
    {
        public float AvoidanceRadius;

        [ReadOnly] public NativeArray<Vector3> AutonomousPositions;
        [ReadOnly] public NativeArray<Vector3> ObstaclePositions;
        public NativeArray<Vector3> ModifiedDirections;

        public void Execute(int i)
        {
            Vector3 currentPosition = AutonomousPositions[i];
            Vector3 currentDirection = ModifiedDirections[i];

            for (int j = 0; j < ObstaclePositions.Length; ++j)
            {
                float dist = (ObstaclePositions[j] - currentPosition).magnitude;
                if (dist < AvoidanceRadius)
                {
                    Vector3 targetDirection = (currentPosition - ObstaclePositions[j]).normalized;
                    ModifiedDirections[i] += targetDirection * AvoidanceRadius;
                    ModifiedDirections[i].Normalize();
                }
            }
        }
    }

    [BurstCompile]
    public struct SeparationWithEnemiesJob : IJobParallelFor
    {
        public float SeparationDistance;
        public float SeparationWeight;

        [ReadOnly] public NativeArray<Vector3> BoidPositions;
        [ReadOnly] public NativeArray<Vector3> EnemyPositions;
        public NativeArray<Vector3> ModifiedDirections;
        public NativeArray<float> ModifiedSpeeds;

        public void Execute(int i)
        {
            Vector3 currentPosition = BoidPositions[i];

            for (int j = 0; j < EnemyPositions.Length; ++j)
            {
                float dist = (EnemyPositions[j] - currentPosition).magnitude;
                if (dist < SeparationDistance)
                {
                    Vector3 targetDirection = (currentPosition - EnemyPositions[j]).normalized;
                    ModifiedDirections[i] += targetDirection;
                    ModifiedDirections[i].Normalize();

                    ModifiedSpeeds[i] += dist * SeparationWeight;
                    ModifiedSpeeds[i] /= 2.0f;
                }
            }
        }
    }


}
