using AStar;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
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
        [ReadOnly] public NativeArray<int> nodeIndicesDestinations;
        [ReadOnly] public NativeArray<NodeTransform> nodes;
        [ReadOnly] public float speed;
        [ReadOnly] public float dt;

        public void Execute(int index, TransformAccess transform)
        {
            int nodeIndex = nodeIndicesDestinations[index];

            if (nodeIndex < 0)
                return;

            Vector3 offset = nodes[nodeIndex].Pos - transform.position;
            transform.position = transform.position + (offset.normalized * (speed * dt));
        }
    }

    [BurstCompile]
    private struct CalculateStartEndPosJob : IJobParallelForTransform
    {
        [ReadOnly] public Vector3 gridBoundsCenter;
        [ReadOnly] public Vector3 gridBoundsExtents;
        [ReadOnly] public float nodeSize;
        [ReadOnly] public bool isGridCreated;
        [ReadOnly] public int gridWidth;

        [ReadOnly] public Random rand;
        [ReadOnly] public NativeArray<Vector3> origEndPositions;

        // might not need these two below

        [WriteOnly] public NativeArray<Vector3> startPositions;
        [WriteOnly] public NativeArray<Vector3> endPositions;

        [WriteOnly] public NativeArray<int> startPositionsIndices;
        [WriteOnly] public NativeArray<int> endPositionsIndices;

        public void Execute(int index, TransformAccess transform)
        {
            Vector3 startPos = transform.position;
            Vector3 endPos = origEndPositions[rand.NextInt(0, origEndPositions.Length - 1)];

            startPositions[index] = startPos;
            endPositions[index] = endPos;

            startPositionsIndices[index] = PosToNodeIndex(startPos);
            endPositionsIndices[index] = PosToNodeIndex(endPos);
        }

        public int PosToNodeIndex(Vector3 pos)
        {
            int index = -1;

            Vector3 localPos = pos - gridBoundsCenter;
            Vector3 extents = gridBoundsExtents;

            float restX = extents.x % nodeSize;
            float restY = extents.y % nodeSize;
            float restZ = extents.z % nodeSize;

            float flooredX = extents.x - restX;
            float flooredY = extents.y - restY;
            float flooredZ = extents.z - restZ;

            Vector3 flooredExtents = new Vector3(flooredX, flooredY, flooredZ);

            if (!isGridCreated ||
                localPos.x < -flooredExtents.x || localPos.x > flooredExtents.x ||
                localPos.z < -flooredExtents.z || localPos.z > flooredExtents.z)
                return index;

            // get rid of negative values
            localPos += flooredExtents;

            int row = Mathf.FloorToInt(localPos.z / nodeSize);
            int col = Mathf.FloorToInt(localPos.x / nodeSize);

            return row * gridWidth + col;
        }

        public void Dispose()
        {
            origEndPositions.Dispose();

            startPositions.Dispose();
            endPositions.Dispose();

            startPositionsIndices.Dispose();
            endPositionsIndices.Dispose();
        }
    }

    [BurstCompile]
    private struct FindPathJobParallel : IJobParallelFor
    {
        [ReadOnly] private int numNodes;
        [ReadOnly] private int gridWidth;
        [ReadOnly] private int numNeighbors;

        [ReadOnly] private NativeArray<int> startNodesIndices;
        [ReadOnly] private NativeArray<int> endNodeIndices;

        [ReadOnly] private NativeArray<NodeNeighbor> nodesNeighbors;
        [ReadOnly] private NativeArray<NodeType> nodesTypes;

        [WriteOnly] public NativeArray<int> nextNodesIndices;

        public void Dispose()
        {
            nextNodesIndices.Dispose();
        }

        public FindPathJobParallel(int numNodes, int gridWidth, int numNeighbors, NativeArray<int> startNodesIndices, NativeArray<int> endNodeIndices, NativeArray<NodeNeighbor> nodesNeighbors, NativeArray<NodeType> nodesTypes, int numPaths)
        {
            this.numNodes = numNodes;
            this.gridWidth = gridWidth;
            this.numNeighbors = numNeighbors;

            this.startNodesIndices = startNodesIndices;
            this.endNodeIndices = endNodeIndices;

            this.nodesNeighbors = nodesNeighbors;
            this.nodesTypes = nodesTypes;

            this.nextNodesIndices = new NativeArray<int>(numPaths, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);
        }

        public void Execute(int index)
        {
            NativeArray<NodePathFindInfo> nodesInfo = new NativeArray<NodePathFindInfo>(numNodes, Allocator.Temp);
            NativeList<int> pathResultNodesIndices = new NativeList<int>(128, Allocator.Temp);
            NativeList<int> openSet = new NativeList<int>(numNodes / 4, Allocator.Temp);

            NativeArray<byte> closedSet = new NativeArray<byte>(numNodes, Allocator.Temp);

            int startNodeIndex = startNodesIndices[index];
            int endNodeIndex = endNodeIndices[index];

            // when an index is invalid, it's set to -1
            if (startNodeIndex < 0 || endNodeIndex < 0)
            {
                pathResultNodesIndices.Add(-1);
                return;
            }

            NativeArray<byte> openSetContains = new NativeArray<byte>(numNodes, Allocator.Temp);

            // set the info for the first node
            nodesInfo[startNodeIndex] = new NodePathFindInfo(0, GetHeuristic(startNodeIndex, endNodeIndex), -1);
            openSet.AddNoResize(startNodeIndex);

            while (openSet.Length > 0)
            {
                int currNodeIndex = PopLowestFCostNodeIndexFromOpenSet(openSet, nodesInfo);

                // we've reached the goal
                if (currNodeIndex == endNodeIndex)
                {
                    pathResultNodesIndices.Add(currNodeIndex);
                    ReconstructPath(index, startNodeIndex, pathResultNodesIndices, nodesInfo);

                    nodesInfo.Dispose();
                    openSetContains.Dispose();
                    openSet.Dispose();
                    closedSet.Dispose();
                    pathResultNodesIndices.Dispose();
                    return;
                }

                // add it to the closed set by setting a flag at its index
                closedSet[currNodeIndex] = 1;
                NodePathFindInfo currNodeInfo = nodesInfo[currNodeIndex];

                // go over the neighbors
                int start = currNodeIndex * numNeighbors;
                int end = start + numNeighbors;

                for (int i = start; i < end; i++)
                {
                    int neighborIndex = nodesNeighbors[i].neighborIndex;
                    bool valid = nodesNeighbors[i].isValid;

                    // if it does not have neighbor or was already expanded
                    if (!valid || closedSet[neighborIndex] == 1)
                        continue;

                    NodeType nodeType = nodesTypes[neighborIndex];

                    // can't be walked by
                    if ((int)nodeType > 0)
                        continue;

                    NodePathFindInfo neighborNodeInfo = nodesInfo[neighborIndex];
                    int newGCost = currNodeInfo.gCost + GetHeuristic(currNodeIndex, neighborIndex);

                    // not in open set
                    if (openSetContains[neighborIndex] != 1)
                    {
                        // update parent, costs, and add to the open set
                        neighborNodeInfo.gCost = newGCost;
                        neighborNodeInfo.hCost = GetHeuristic(neighborIndex, endNodeIndex);
                        neighborNodeInfo.parentNodeIndex = currNodeIndex;

                        nodesInfo[neighborIndex] = neighborNodeInfo;
                        openSet.AddNoResize(neighborIndex);
                        openSetContains[neighborIndex] = 1;
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

            pathResultNodesIndices.Add(-1);
            ReconstructPath(index, startNodeIndex, pathResultNodesIndices, nodesInfo);

            nodesInfo.Dispose();
            openSetContains.Dispose();
            openSet.Dispose();
            closedSet.Dispose();
            pathResultNodesIndices.Dispose();
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

        private void ReconstructPath(int index, int startNodeIndex, NativeList<int> pathResultNodesIndices, NativeArray<NodePathFindInfo> nodesInfo)
        {
            if (pathResultNodesIndices.Length == 0 || pathResultNodesIndices[0] == -1)
            {
                nextNodesIndices[index] = -1;
                return;
            }

            int currNode = pathResultNodesIndices[0];

            while (currNode != startNodeIndex)
            {
                int parentNodeIndex = nodesInfo[currNode].parentNodeIndex;
                currNode = parentNodeIndex;

                pathResultNodesIndices.Add(parentNodeIndex);
            }

            // the last node on the path is the following one (its reversed)
            nextNodesIndices[index] = pathResultNodesIndices.ElementAt(pathResultNodesIndices.Length - 1);
        }
    }

    public GameObject agentPrefab;
    public float agentsSpeed = 10.0f;
    public int quantity;

    public Transform[] endPositions;

    private Transform[] agentsTransforms;
    private TransformAccessArray agentsTransAcc;

    private void Start()
    {
        Spawn();
    }

    private void Update()
    {
        NativeArray<Vector3> origEndPositions = new NativeArray<Vector3>(endPositions.Length, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);
        for (int i = 0; i < endPositions.Length; i++)
            origEndPositions[i] = endPositions[i].position;

        GridMaster gm = GridMaster.Instance;

        CalculateStartEndPosJob calcStartEndJob = new CalculateStartEndPosJob()
        {
            gridBoundsCenter = gm.Bounds.center,
            gridBoundsExtents = gm.Bounds.extents,
            nodeSize = GridMaster.NodeSize,
            isGridCreated = gm.IsGridCreated,
            gridWidth = gm.GridWidth,

            origEndPositions = origEndPositions,
            startPositions = new NativeArray<Vector3>(quantity, Allocator.TempJob, NativeArrayOptions.UninitializedMemory),
            endPositions = new NativeArray<Vector3>(quantity, Allocator.TempJob, NativeArrayOptions.UninitializedMemory),

            startPositionsIndices = new NativeArray<int>(quantity, Allocator.TempJob, NativeArrayOptions.UninitializedMemory),
            endPositionsIndices = new NativeArray<int>(quantity, Allocator.TempJob, NativeArrayOptions.UninitializedMemory),
            rand = new Random(0x6E624EB7u)
        };

        JobHandle calcStartEndHandle = calcStartEndJob.Schedule(agentsTransAcc);

        FindPathJobParallel findPathsJob = new FindPathJobParallel(gm.GridWidth * gm.GridDepth, gm.GridWidth, 4, calcStartEndJob.startPositionsIndices, calcStartEndJob.endPositionsIndices, gm.NodesNeighbors, gm.NodesTypes, quantity);
        JobHandle findPathsHandle = findPathsJob.Schedule(quantity, 4, calcStartEndHandle);

        MoveAgentsJob moveJob = new MoveAgentsJob()
        {
            nodeIndicesDestinations = findPathsJob.nextNodesIndices,
            nodes = gm.NodesTransforms,
            speed = agentsSpeed,
            dt = Time.deltaTime
        };

        JobHandle moveHandle = moveJob.Schedule(agentsTransAcc, findPathsHandle);
        moveHandle.Complete();

        findPathsJob.Dispose();
        calcStartEndJob.Dispose();
    }

    private void OnDestroy()
    {
        agentsTransAcc.Dispose();
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

        agentsTransAcc = new TransformAccessArray(agentsTransforms);
    }
}