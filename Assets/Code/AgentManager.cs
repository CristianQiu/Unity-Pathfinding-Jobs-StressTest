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
        [ReadOnly] public NativeArray<NodeTransform> nodes;
        [ReadOnly] public NativeArray<int> destinationNodesIndices;
        [ReadOnly] public float speed;
        [ReadOnly] public float dt;

        public void Execute(int index, TransformAccess transform)
        {
            int nodeIndex = destinationNodesIndices[index];

            if (nodeIndex < 0)
                return;

            Vector3 offset = nodes[nodeIndex].Pos - transform.position;
            Vector3 delta = offset.normalized * (speed * dt);

            Vector3.ClampMagnitude(delta, offset.magnitude);

            transform.position = transform.position + delta;
        }
    }

    [BurstCompile]
    private struct CalculateStartEndPosJob : IJobParallelForTransform
    {
        [ReadOnly] public NativeArray<Vector3> endPositionsToChooseFrom;
        [WriteOnly] public NativeArray<Vector3> endPositions;
        [WriteOnly] public NativeArray<Vector3> startPositions;
        [ReadOnly] public Random rand;

        public void Execute(int index, TransformAccess transform)
        {
            // choose a random end position from all the available ones
            endPositions[index] = endPositionsToChooseFrom[rand.NextInt(0, endPositionsToChooseFrom.Length - 1)];
            startPositions[index] = transform.position;
        }

        public void Dispose()
        {
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

    private NativeArray<Vector3> endPositionsToChooseFrom;

    private void Start()
    {
        Spawn();

        endPositionsToChooseFrom = new NativeArray<Vector3>(endPositions.Length, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
        for (int i = 0; i < endPositions.Length; i++)
            endPositionsToChooseFrom[i] = endPositions[i].position;
    }

    private void Update()
    {
        CalculateStartEndPosJob calcStartEndJob = new CalculateStartEndPosJob()
        {
            endPositionsToChooseFrom = endPositionsToChooseFrom,
            startPositions = new NativeArray<Vector3>(quantity, Allocator.TempJob, NativeArrayOptions.UninitializedMemory),
            endPositions = new NativeArray<Vector3>(quantity, Allocator.TempJob, NativeArrayOptions.UninitializedMemory),
            rand = new Random(0x6E624EB7u)
        };

        // I dont like having several completes instead of using dependencies, but I would need some
        // rework due to being blocked by code that I can not currently jobify. Besides, there is
        // probably a better approach than scheduling hundreds of paths one by one from the main thread
        JobHandle calcStartEndHandle = calcStartEndJob.Schedule(transAcc);

        PathFinder pf = PathFinder.Instance;
        GridMaster gm = GridMaster.Instance;

        calcStartEndHandle.Complete();

        NativeArray<int> nextNodesIndices = new NativeArray<int>(quantity, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);
        pf.FindNextNodeIndexOnPathsBatch(calcStartEndJob.startPositions, calcStartEndJob.endPositions, nextNodesIndices);

        MoveAgentsJob moveJob = new MoveAgentsJob()
        {
            nodes = gm.NodesTransforms,
            destinationNodesIndices = nextNodesIndices,
            speed = agentsSpeed,
            dt = Time.deltaTime
        };

        JobHandle moveHandle = moveJob.Schedule(transAcc);
        moveHandle.Complete();

        calcStartEndJob.Dispose();
        nextNodesIndices.Dispose();
    }

    private void OnDestroy()
    {
        endPositionsToChooseFrom.Dispose();
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
            float x = t * 90.0f;
            x -= 45.0f;

            // scatter the spawn
            agentsTransforms[i].position = transform.position + new Vector3(x, 0.0f, UnityEngine.Random.Range(-3.0f, 3.0f));
        }

        transAcc = new TransformAccessArray(agentsTransforms);
    }
}