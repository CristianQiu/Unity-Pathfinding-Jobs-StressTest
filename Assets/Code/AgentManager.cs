using AStar;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using UnityEngine;
using UnityEngine.Jobs;
using Random = Unity.Mathematics.Random;

/// <summary>
/// Class to make a quick stress test of the jobified pathfinding.
/// </summary>
public class AgentManager : MonoBehaviour
{
    [BurstCompile]
    private struct MoveAgentsJob : IJobParallelForTransform
    {
        [ReadOnly] public NativeArray<Vector3> destinations;
        [ReadOnly] public float speed;
        [ReadOnly] public float dt;

        public void Execute(int index, TransformAccess transform)
        {
            //Vector3 offset = destinations[index] - transform.position;
            //transform.position = transform.position + (offset.normalized * (speed * dt));
        }
    }

    [BurstCompile]
    private struct CalculateStartEndPosJob : IJobParallelForTransform
    {
        [ReadOnly] public NativeArray<Vector3> origEndPositions;
        [WriteOnly] public NativeArray<Vector3> startPositions;
        [WriteOnly] public NativeArray<Vector3> endPositions;
        [ReadOnly] public Random rand;

        public void Execute(int index, TransformAccess transform)
        {
            startPositions[index] = transform.position;

            // choose a random end position from all the available ones
            endPositions[index] = origEndPositions[rand.NextInt(0, origEndPositions.Length - 1)];
        }

        public void Dispose()
        {
            origEndPositions.Dispose();
            startPositions.Dispose();
            endPositions.Dispose();
        }
    }

    public Transform[] endPositions;
    public GameObject agentPrefab;

    public float agentsSpeed = 10.0f;
    public int quantity;

    private Transform[] agentsTransforms;
    private TransformAccessArray transAcc;

    private void Start()
    {
        Spawn();
    }

    private void Update()
    {
        NativeArray<Vector3> origEndPositions = new NativeArray<Vector3>(endPositions.Length, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);
        for (int i = 0; i < endPositions.Length; i++)
            origEndPositions[i] = endPositions[i].position;

        CalculateStartEndPosJob calcStartEndJob = new CalculateStartEndPosJob()
        {
            origEndPositions = origEndPositions,
            startPositions = new NativeArray<Vector3>(quantity, Allocator.TempJob, NativeArrayOptions.UninitializedMemory),
            endPositions = new NativeArray<Vector3>(quantity, Allocator.TempJob, NativeArrayOptions.UninitializedMemory),
            rand = new Random(0x6E624EB7u)
        };

        // I dont like having several completes instead of using dependencies, but I would need some
        // rework due to being blocked by code that I can not jobify. Besides, there is probably a
        // better approach than scheduling hundreds of paths one by one from the main thread.
        JobHandle calcStartEndHandle = calcStartEndJob.Schedule(transAcc);

        MoveAgentsJob moveJob = new MoveAgentsJob()
        {
            destinations = calcStartEndJob.endPositions,
            speed = agentsSpeed,
            dt = Time.deltaTime
        };

        PathFinder pf = PathFinder.Instance;

        calcStartEndHandle.Complete();

        pf.FindNextPointOnPathsBatch(calcStartEndJob.startPositions, calcStartEndJob.endPositions);

        JobHandle moveHandle = moveJob.Schedule(transAcc);
        moveHandle.Complete();

        calcStartEndJob.Dispose();
    }

    private void OnDestroy()
    {
        transAcc.Dispose();
    }

    private void Spawn()
    {
        agentsTransforms = new Transform[quantity];

        for (int i = 0; i < quantity; i++)
        {
            // having no parent is actually very important. Sharing the parent will prevent the job
            // system to split calculations across threads
            agentsTransforms[i] = Instantiate(agentPrefab, null).transform;

            // spawn along x= -45 to +45
            float t = (float)i / (float)quantity;
            t *= 90.0f;
            t -= 45.0f;

            // scatter the spawn along the x axis to avoid excessive overlapping
            agentsTransforms[i].position = transform.position + new Vector3(t, 0.0f, 0.0f);
        }

        transAcc = new TransformAccessArray(agentsTransforms);
    }
}