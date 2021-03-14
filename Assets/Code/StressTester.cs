using AStar;
using TMPro;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEditor;
using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.Jobs;
using UnityEngine.SceneManagement;
using UnityEngine.UI;
using Random = Unity.Mathematics.Random;

/// <summary>
/// Class to make a quick stress test of the jobified pathfinding.
/// </summary>
[DefaultExecutionOrder(1)] // < Make sure the AStar system is initialized so we can safely do stuff on start, like building the grid.
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
    private struct FindPathJobParallel : IJobParallelFor
    {
        [WriteOnly] public NativeArray<int> nextNodesIndices;

        [ReadOnly] public NativeArray<int> startNodesIndices;
        [ReadOnly] public NativeArray<int> endNodeIndices;

        [ReadOnly] public NativeArray<NodeNeighbor> nodesNeighbors;
        [ReadOnly] public NativeArray<NodeType> nodesTypes;

        public int numNodes;
        public int gridWidth;
        public int numNeighbors;

        public void Execute(int index)
        {
            int startNodeIndex = startNodesIndices[index];
            int endNodeIndex = endNodeIndices[index];

            // when an index is invalid, it's set to -1
            if (startNodeIndex < 0 || endNodeIndex < 0)
            {
                nextNodesIndices[index] = -1;
                return;
            }

            NativeArray<NodePathFindInfo> nodesInfo = new NativeArray<NodePathFindInfo>(numNodes, Allocator.Temp);
            NativeList<int> openSet = new NativeList<int>(272, Allocator.Temp);
            NativeBitArray openSetContains = new NativeBitArray(numNodes, Allocator.Temp);
            NativeBitArray closedSet = new NativeBitArray(numNodes, Allocator.Temp);

            // set the info for the first node
            nodesInfo[startNodeIndex] = new NodePathFindInfo(0, GetHeuristic(startNodeIndex, endNodeIndex), -1);
            openSet.AddNoResize(startNodeIndex);

            while (openSet.Length > 0)
            {
                int currNodeIndex = PopLowestFCostNodeIndexFromOpenSet(openSet, nodesInfo);

                // we've reached the goal
                if (currNodeIndex == endNodeIndex)
                {
                    ReconstructPath(index, startNodeIndex, nodesInfo);
                    return;
                }

                // add it to the closed set by setting a flag at its index
                closedSet.Set(currNodeIndex, true);
                NodePathFindInfo currNodeInfo = nodesInfo[currNodeIndex];

                // go over the neighbors
                int start = currNodeIndex * numNeighbors;
                int end = start + numNeighbors;

                for (int i = start; i < end; i++)
                {
                    int neighborIndex = nodesNeighbors[i].neighborIndex;
                    bool valid = nodesNeighbors[i].isValid;

                    // if it does not have neighbor or was already expanded
                    if (!valid || closedSet.IsSet(neighborIndex))
                        continue;

                    NodeType nodeType = nodesTypes[neighborIndex];

                    // can't be walked by
                    if ((byte)nodeType > 0)
                        continue;

                    NodePathFindInfo neighborNodeInfo = nodesInfo[neighborIndex];
                    int newGCost = currNodeInfo.gCost + GetHeuristic(currNodeIndex, neighborIndex);

                    // not in open set
                    if (!openSetContains.IsSet(neighborIndex))
                    {
                        // update parent, costs, and add to the open set
                        neighborNodeInfo.gCost = newGCost;
                        neighborNodeInfo.hCost = GetHeuristic(neighborIndex, endNodeIndex);
                        neighborNodeInfo.parentNodeIndex = currNodeIndex;

                        nodesInfo[neighborIndex] = neighborNodeInfo;
                        openSet.AddNoResize(neighborIndex);
                        openSetContains.Set(neighborIndex, true);
                    }
                    else if (newGCost < neighborNodeInfo.gCost)
                    {
                        // update parent, and gCost (hCost is already calculated)
                        neighborNodeInfo.gCost = newGCost;
                        neighborNodeInfo.parentNodeIndex = currNodeIndex;

                        nodesInfo[neighborIndex] = neighborNodeInfo;
                    }
                }
            }

            nextNodesIndices[index] = -1;
        }

        private int PopLowestFCostNodeIndexFromOpenSet(NativeList<int> openSet, NativeArray<NodePathFindInfo> nodesInfo)
        {
            int foundAtIndex = -1;
            int lowestIndex = -1;

            int lowestFCostHCost = int.MaxValue;
            int lowestFCostVal = int.MaxValue;

            for (int i = 0; i < openSet.Length; i++)
            {
                int currNodeIndex = openSet[i];
                NodePathFindInfo info = nodesInfo[currNodeIndex];

                if (info.FCost < lowestFCostVal || info.FCost == lowestFCostVal && info.hCost < lowestFCostHCost)
                {
                    foundAtIndex = i;

                    lowestIndex = currNodeIndex;
                    lowestFCostHCost = info.hCost;
                    lowestFCostVal = info.FCost;
                }
            }

            openSet.RemoveAtSwapBack(foundAtIndex);

            return lowestIndex;
        }

        private int GetHeuristic(int fromNodeIndex, int toNodeIndex)
        {
            int fromRow = fromNodeIndex / gridWidth;
            int fromCol = fromNodeIndex % gridWidth;

            int toRow = toNodeIndex / gridWidth;
            int toCol = toNodeIndex % gridWidth;

            int rowOffset = math.max(fromRow, toRow) - math.min(fromRow, toRow);
            int colOffset = math.max(fromCol, toCol) - math.min(fromCol, toCol);

            return rowOffset + colOffset;
        }

        private void ReconstructPath(int index, int startNodeIndex, NativeArray<NodePathFindInfo> nodesInfo)
        {
            int currNode = endNodeIndices[index];
            int nextNode = -1;

            while (currNode != startNodeIndex)
            {
                nextNode = currNode;
                int parentNodeIndex = nodesInfo[currNode].parentNodeIndex;
                currNode = parentNodeIndex;
            }

            nextNodesIndices[index] = nextNode;
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

        Debug.Log("MB temp allocator " + (UnityEngine.Profiling.Profiler.GetTempAllocatorSize() / (1024 * 1024)));

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
            handles[i] = Pathfinder.ScheduleFindPath(startPositionsIndices, endPositionsIndices, nextNodesIndices, i, deps);
        }

        //deps = JobHandle.CombineDependencies(handles);
        JobHandle.CompleteAll(handles);
        handles.Dispose();

        //deps = new FindPathJobParallel()
        //{
        //    nextNodesIndices = nextNodesIndices,
        //    startNodesIndices = startPositionsIndices,
        //    endNodeIndices = endPositionsIndices,
        //    nodesNeighbors = gm.NodesNeighbors,
        //    nodesTypes = gm.NodesTypes,
        //    numNodes = gm.GridWidth * gm.GridDepth,
        //    gridWidth = gm.GridWidth,
        //    numNeighbors = GridMaster.NodeNumNeighbors,
        //}
        //.Schedule(quantity, 1, deps);

        deps = new MoveAgentsJob()
        {
            nodeIndicesDestinations = nextNodesIndices,
            nodes = gm.NodesTransforms,
            speed = agentsSpeed,
            dt = dt,
        }
        .Schedule(agentsTransAcc, deps);

        UpdateCamPivot(dt);

        JobHandle disposeHandle = startPositionsIndices.Dispose(deps);
        disposeHandle = JobHandle.CombineDependencies(disposeHandle, endPositionsIndices.Dispose(deps), nextNodesIndices.Dispose(deps));

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