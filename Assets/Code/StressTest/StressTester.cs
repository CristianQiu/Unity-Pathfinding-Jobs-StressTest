using AStar;
using TMPro;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using UnityEditor;
using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.Jobs;
using UnityEngine.Profiling;
using UnityEngine.SceneManagement;
using UnityEngine.UI;
using Random = Unity.Mathematics.Random;

/// <summary>
/// Class to make a quick stress test of the jobified pathfinding.
/// </summary>
[DefaultExecutionOrder(1)] // < Quick way to ensure the AStar system & debug is initialized so we can safely do stuff on start, like building the grid.
public class StressTester : MonoBehaviour
{
    #region Jobs

    [BurstCompile]
    private struct CalculateStartEndPosJob : IJobParallelForTransform
    {
        [WriteOnly] public NativeArray<int> startPositionsIndices;
        [WriteOnly] public NativeArray<int> endPositionsIndices;

        [ReadOnly] public NativeArray<Vector3> endPositionsToChooseFrom;

        public Vector3 gridBoundsCenter;
        public Vector3 gridBoundsExtents;
        public bool isGridCreated;
        public int gridWidth;
        public float nodeSize;

        public Random rand;

        public void Execute(int index, TransformAccess transform)
        {
            Vector3 startPos = transform.position;
            Vector3 endPos = endPositionsToChooseFrom[rand.NextInt(0, endPositionsToChooseFrom.Length - 1)];

            startPositionsIndices[index] = PosToNodeIndex(startPos);
            endPositionsIndices[index] = PosToNodeIndex(endPos);
        }

        private int PosToNodeIndex(Vector3 pos)
        {
            int index = -1;

            Vector3 localPos = pos - gridBoundsCenter;
            Vector3 extents = gridBoundsExtents;

            float restX = extents.x % nodeSize;
            float restZ = extents.z % nodeSize;

            float flooredX = extents.x - restX;
            float flooredZ = extents.z - restZ;

            Vector3 flooredExtents = new Vector3(flooredX, 0.0f, flooredZ);

            if (!isGridCreated ||
                localPos.x < -flooredExtents.x || localPos.x > flooredExtents.x ||
                localPos.z < -flooredExtents.z || localPos.z > flooredExtents.z)
                return index;

            localPos += flooredExtents;

            int row = Mathf.FloorToInt(localPos.z / nodeSize);
            int col = Mathf.FloorToInt(localPos.x / nodeSize);

            return row * gridWidth + col;
        }
    }

    [BurstCompile]
    private struct MoveAgentsJob : IJobParallelForTransform
    {
        [ReadOnly] public NativeArray<int> nodeIndicesDestinations;
        [ReadOnly] public NativeArray<NodeTransform> nodes;

        public float speed;
        public float dt;

        public void Execute(int index, TransformAccess transform)
        {
            int nodeIndex = nodeIndicesDestinations[index];

            if (nodeIndex < 0)
                return;

            Vector3 offset = nodes[nodeIndex].Pos + new Vector3(0.0f, transform.localScale.y * 0.5f, 0.0f) - transform.position;
            transform.position = transform.position + offset.normalized * (speed * dt);
        }
    }

    #endregion

    #region Private Attributes

    [Header("Agents")]
    [SerializeField] private GameObject agentPrefab = null;
    [SerializeField] private float agentsSpeed = 10.0f;
    [SerializeField] private Transform[] endPositions = null;

    private int quantity;

    private Transform[] agentsTransforms;
    private TransformAccessArray agentsTransAcc;
    private NativeArray<Vector3> endPositionsToChooseFrom;

    private CustomSampler sampler = CustomSampler.Create("StressTester.CustomSample");

    #endregion

    #region MonoBehaviour Methods

    private void Start()
    {
        quantity = PlayerPrefs.GetInt("quantity", (int)quantitySlider.value);
        agentsText.text = "Agents: " + quantity.ToString();
        quantitySlider.SetValueWithoutNotify(quantity);

        GridMaster.Instance.CreateGrid();

        SpawnAgents();
        SpawnGraphy();

        endPositionsToChooseFrom = new NativeArray<Vector3>(endPositions.Length, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
        for (int i = 0; i < endPositions.Length; i++)
            endPositionsToChooseFrom[i] = endPositions[i].position;
    }

    private void Update()
    {
        float dt = Time.deltaTime;
        GridMaster gm = GridMaster.Instance;

        NativeArray<int> startPositionsIndices = new NativeArray<int>(quantity, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);
        NativeArray<int> endPositionsIndices = new NativeArray<int>(quantity, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);

        JobHandle deps = new CalculateStartEndPosJob()
        {
            startPositionsIndices = startPositionsIndices,
            endPositionsIndices = endPositionsIndices,
            endPositionsToChooseFrom = endPositionsToChooseFrom,
            gridBoundsCenter = gm.Bounds.center,
            gridBoundsExtents = gm.Bounds.extents,
            isGridCreated = gm.IsGridCreated,
            gridWidth = gm.GridWidth,
            nodeSize = GridMaster.NodeSize,
            rand = new Random(0x6E624EB7u),
        }
        .Schedule(agentsTransAcc);

        NativeArray<int> nextNodesIndices = new NativeArray<int>(quantity, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);
        NativeArray<JobHandle> handles = new NativeArray<JobHandle>(quantity, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);

        for (int i = 0; i < quantity; i++)
        {
            handles[i] = Pathfinder.ScheduleFindPath(gm, startPositionsIndices, endPositionsIndices, nextNodesIndices, i, deps);

            // There's some interesting balancing oportunity here, the greater the number, the
            // lesser the overhead the main thread will have, and so it will be able to finish
            // scheduling the rest of the task faster, but the greater the number the more time
            // before the workers actually start performing the tasks, so a lot of time is wasted
            // while the main thread keeps "enqueuing" the tasks and the workers are in idle. If
            // you're benchmarking this, either make sure the jobsdebugger, safety checks and leak
            // detection is off, or ideally, make a profiled build.
            if ((i + 1) % 256 == 0)
                JobHandle.ScheduleBatchedJobs();
        }

        // I feel like combining all of these (tested with 2.5k+ paths) is bottlenecking, perhaps
        // there's a way of reordering all the stuff so that it makes a minimal impact, rather than
        // an obvious bottleneck when there's very little work on the paths but a lot of paths. I
        // suggest profiling in build and wait all agents to reach their destinations, then see the
        // main thread in the profiler.
        deps = JobHandle.CombineDependencies(handles);
        deps = new MoveAgentsJob()
        {
            nodeIndicesDestinations = nextNodesIndices,
            nodes = gm.NodesTransforms,
            speed = agentsSpeed,
            dt = dt,
        }
        .Schedule(agentsTransAcc, deps);

        JobHandle disposeHandle = default(JobHandle);
        disposeHandle = JobHandle.CombineDependencies(startPositionsIndices.Dispose(disposeHandle), endPositionsIndices.Dispose(disposeHandle), handles.Dispose(disposeHandle));
        disposeHandle = JobHandle.CombineDependencies(disposeHandle, nextNodesIndices.Dispose(deps));

        UpdateCamPivot(dt);

        JobHandle.CompleteAll(ref deps, ref disposeHandle);

#if !UNITY_EDITOR
        if (Keyboard.current.escapeKey.wasPressedThisFrame)
            Application.Quit();
#else
        if (Keyboard.current.escapeKey.wasPressedThisFrame)
            EditorApplication.isPlaying = false;
#endif
    }

    private void OnDestroy()
    {
        if (agentsTransAcc.isCreated)
            agentsTransAcc.Dispose();
        if (endPositionsToChooseFrom.IsCreated)
            endPositionsToChooseFrom.Dispose();
    }

    #endregion

    #region Methods

    private void SpawnAgents()
    {
        agentsTransforms = new Transform[quantity];

        for (int i = 0; i < quantity; i++)
        {
            // having no parent is actually important. Sharing the parent will prevent the job
            // system to split calculations across threads properly
            agentsTransforms[i] = Instantiate(agentPrefab, null).transform;

            float t = (float)i / (float)(quantity - 1);

            float x = t * 90.0f;
            x -= 45.0f;

            float z = UnityEngine.Random.Range(-4.0f, 4.0f);

            // scatter the spawn along the x/z axis to avoid excessive overlapping
            agentsTransforms[i].position = transform.position + new Vector3(x, 0.0f, z);

            // uncomment for color variation but sacrify instancing
            //agentsTransforms[i].GetComponentInChildren<MeshRenderer>().material.color = new Color(t, t * (z * 0.25f), z * 0.25f, 1.0f);

            float sy = UnityEngine.Random.Range(0.4f, 1.0f);
            float sxz = UnityEngine.Random.Range(0.4f, 0.8f);

            agentsTransforms[i].localScale = new Vector3(sxz, sy, sxz);
        }

        agentsTransAcc = new TransformAccessArray(agentsTransforms);
    }

    #endregion

    #region Cam

    [Header("Camera")]
    [SerializeField] private Transform camPivot = null;
    [SerializeField] private Transform camPivotEnd = null;

    private void UpdateCamPivot(float dt)
    {
        Vector3 inc = camPivotEnd.position - camPivot.position;
        inc.y = 0.0f;

        Vector3 delta = inc.normalized * (agentsSpeed * 1.2f * dt);
        delta = Vector3.ClampMagnitude(delta, inc.magnitude);

        camPivot.transform.position += delta;
    }

    #endregion

    #region UI

    [Header("UI")]
    [SerializeField] private GameObject graphyPrefab = null;
    [SerializeField] private Slider quantitySlider = null;
    [SerializeField] private TextMeshProUGUI agentsText = null;

    private void SpawnGraphy()
    {
        Instantiate(graphyPrefab);
    }

    public void OnRestartClicked()
    {
        PlayerPrefs.SetInt("quantity", (int)quantitySlider.value);
        SceneManager.LoadScene(SceneManager.GetActiveScene().buildIndex);
    }

    #endregion
}