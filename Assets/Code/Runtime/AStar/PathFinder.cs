using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;
using UnityLibrary;
using Logger = UnityLibrary.Logger;

namespace AStar
{
    /// <summary>
    /// Class to do pathfinding tasks using the nodes information from the GridMaster.
    /// </summary>
    public class PathFinder : MonoBehaviourSingleton<PathFinder>
    {
        #region Pathfinding Jobs

        /// <summary>
        /// The job in charge of finding the path.
        /// </summary>
        [BurstCompile]
        private struct FindPathJob : IJob
        {
            [ReadOnly] private readonly int numNodes;
            [ReadOnly] private readonly int gridWidth;
            [ReadOnly] private readonly int numNeighbors;

            [ReadOnly] private readonly int startNodeIndex;
            [ReadOnly] private readonly int endNodeIndex;

            [ReadOnly] private readonly NativeArray<NodeNeighbor> nodesNeighbors;
            [ReadOnly] private readonly NativeArray<NodeType> nodesTypes;

            private NativeArray<NodePathFindInfo> nodesInfo;

            public NativeList<int> openSet;
            public NativeArray<byte> closedSet;

            public NativeList<int> pathResultNodesIndices;

            public FindPathJob(int numNodes, int gridWidth, int numNeighbors, int startNodeIndex, int endNodeIndex, NativeArray<NodeNeighbor> nodesNeighbors, NativeArray<NodeType> nodesTypes)
            {
                this.numNodes = numNodes;
                this.gridWidth = gridWidth;
                this.numNeighbors = numNeighbors;

                this.startNodeIndex = startNodeIndex;
                this.endNodeIndex = endNodeIndex;

                this.nodesNeighbors = nodesNeighbors;
                this.nodesTypes = nodesTypes;

                nodesInfo = new NativeArray<NodePathFindInfo>(numNodes, Allocator.TempJob);

                // Note: could make these just temp by using them in the execute method, although I
                // want them available so that I can debug the opened and closed nodes
                openSet = new NativeList<int>(numNodes, Allocator.TempJob);
                closedSet = new NativeArray<byte>(numNodes, Allocator.TempJob);

                pathResultNodesIndices = new NativeList<int>(Allocator.TempJob);
            }

            /// <inheritdoc/>
            public void Execute()
            {
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
                    int currNodeIndex = PopLowestFCostNodeIndexFromOpenSet();

                    // we've reached the goal
                    if (currNodeIndex == endNodeIndex)
                    {
                        pathResultNodesIndices.Add(currNodeIndex);
                        ReconstructPath();
                        openSetContains.Dispose();
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
                ReconstructPath();
                openSetContains.Dispose();
            }

            /// <summary>
            /// Pops the lowest FCost node index from the open set.
            /// </summary>
            /// <returns></returns>
            private int PopLowestFCostNodeIndexFromOpenSet()
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

                // if we get to this function we are sure there is always at least one, no need to check
                openSet.RemoveAtSwapBack(foundAtIndex);

                return lowestIndex;
            }

            /// <summary>
            /// Get the hCost from a node to another one.
            /// </summary>
            /// <param name="fromNodeIndex"></param>
            /// <param name="toNodeIndex"></param>
            /// <returns></returns>
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

            /// <summary>
            /// Reconstruct the path.
            /// </summary>
            private void ReconstructPath()
            {
                if (pathResultNodesIndices.Length == 0)
                    return;

                int currNode = pathResultNodesIndices[0];

                while (currNode != startNodeIndex)
                {
                    int parentNodeIndex = nodesInfo[currNode].parentNodeIndex;
                    currNode = parentNodeIndex;

                    pathResultNodesIndices.Add(parentNodeIndex);
                }
            }

            /// <summary>
            /// Deallocates the datastructures used by this job.
            /// </summary>
            public void Dispose()
            {
                openSet.Dispose();
                closedSet.Dispose();
                nodesInfo.Dispose();
                pathResultNodesIndices.Dispose();
            }
        }

        #endregion

        #region Reachable Nodes Job

        /// <summary>
        /// The job used to find a set of reachable nodes given a max depth and a start node.
        /// </summary>
        [BurstCompile]
        private struct FindReachableNodesJob : IJob
        {
            [ReadOnly] private readonly int numNodes;
            [ReadOnly] private readonly int numNeighbors;

            [ReadOnly] private readonly int startNodeIndex;
            [ReadOnly] private readonly int maxDepth;

            [ReadOnly] private readonly NativeArray<NodeNeighbor> nodesNeighbors;
            [ReadOnly] private readonly NativeArray<NodeType> nodesTypes;

            public NativeArray<NodeBreadthFirstSearchInfo> nodesInfo;
            public NativeList<int> reachableNodesList;
            public NativeHashMap<int, int> reachableNodesHashmap;

            public FindReachableNodesJob(int numNodes, int numNeighbors, int startNodeIndex, int maxDepth, NativeArray<NodeType> nodesTypes, NativeArray<NodeNeighbor> nodesNeighbors, NativeList<int> reachableNodesListResult, NativeHashMap<int, int> reachableNodesHashmapResult)
            {
                this.numNodes = numNodes;
                this.numNeighbors = numNeighbors;

                this.startNodeIndex = startNodeIndex;
                this.maxDepth = maxDepth;

                this.nodesTypes = nodesTypes;
                this.nodesNeighbors = nodesNeighbors;

                this.nodesInfo = new NativeArray<NodeBreadthFirstSearchInfo>(numNodes, Allocator.TempJob);
                this.reachableNodesList = reachableNodesListResult;
                this.reachableNodesHashmap = reachableNodesHashmapResult;
            }

            /// <inheritdoc/>
            public void Execute()
            {
                NativeList<int> openSet = new NativeList<int>(Allocator.Temp);
                NativeArray<byte> reachableNodesContains = new NativeArray<byte>(numNodes, Allocator.Temp);

                // the resulting list / hashmap where we are introducing the result, so clear it
                reachableNodesList.Clear();
                reachableNodesHashmap.Clear();

                // add the first node
                NodeBreadthFirstSearchInfo currNodeInfo = new NodeBreadthFirstSearchInfo(0, -1);
                nodesInfo[startNodeIndex] = currNodeInfo;
                openSet.Add(startNodeIndex);

                // while we've got stuff queued
                while (openSet.Length > 0)
                {
                    int currNodeIndex = openSet[0];
                    openSet.RemoveAt(0);
                    currNodeInfo = nodesInfo[currNodeIndex];

                    int start = currNodeIndex * numNeighbors;
                    int end = start + numNeighbors;

                    // iterate over neighbors
                    for (int i = start; i < end; i++)
                    {
                        int neighborIndex = nodesNeighbors[i].neighborIndex;
                        bool valid = nodesNeighbors[i].isValid;

                        if (!valid)
                            continue;

                        // calculate the new depth
                        int newDepth = currNodeInfo.depth + 1;
                        byte nodeType = (byte)nodesTypes[neighborIndex];

                        // nodes that cant be passed through are not valid either
                        if (newDepth > maxDepth || nodeType > 0)
                            continue;

                        // if the node was not in closed set add it with the depth as is
                        if (reachableNodesContains[neighborIndex] != 1)
                        {
                            NodeBreadthFirstSearchInfo neighborNodeInfo = new NodeBreadthFirstSearchInfo(newDepth, currNodeIndex);
                            nodesInfo[neighborIndex] = neighborNodeInfo;

                            openSet.Add(neighborIndex);
                            reachableNodesContains[neighborIndex] = 1;
                            reachableNodesList.Add(neighborIndex);
                            reachableNodesHashmap.Add(neighborIndex, neighborIndex);
                        }
                        else
                        {
                            // if the node was already reachable and the depth is lesser update
                            NodeBreadthFirstSearchInfo neighborNodeInfo = nodesInfo[neighborIndex];

                            if (newDepth < neighborNodeInfo.depth)
                            {
                                neighborNodeInfo.parentNodeIndex = currNodeIndex;
                                neighborNodeInfo.depth = newDepth;
                                nodesInfo[neighborIndex] = neighborNodeInfo;

                                openSet.Add(neighborIndex);
                            }
                        }
                    }
                }

                openSet.Dispose();
                reachableNodesContains.Dispose();
            }

            /// <summary>
            /// Deallocates the datastructures used by this job.
            /// </summary>
            public void Dispose()
            {
                nodesInfo.Dispose();
            }
        }

        #endregion

#if DEBUG_RENDER

        #region Public Atrributes

        [Header("Shared material")]
        [SerializeField] private Material debugMaterial = null;

        [Header("Movement range debug")]
        public bool debugMovementRange = true;

        public Color32 movementRangeDebugColor = new Color32(87, 85, 255, 255);

        [Header("Pathfinding debug")]
        public bool debugPathfinding = true;

        public Color32 openSetColor = new Color32();
        public Color32 closedSetColor = new Color32();
        public Color32 pathColor = new Color32();

        #endregion

        #region Private Attributes

        private const float NodeVisualDebugSize = GridMaster.NodeSize * 0.625f;
        private const float SmallUpDebugOffsetMovementRange = 0.004f;
        private const float SmallUpDebugOffsetOpenClosedSet = 0.002f;
        private const float SmallUpDebugOffsetPath = 0.003f;

        private const float ScaleMovementRangeNode = 0.33f;
        private const float ScalePathNode = 0.75f;

        private Mesh debugMesh;
        private Batcher movementRangeBatcher;
        private Batcher pathfindBatcher;

        #endregion

#endif

        #region Properties

        protected override bool DestroyOnLoad { get { return true; } }

        #endregion

        #region MonoBehaviour Methods

        protected override void Awake()
        {
            base.Awake();

#if DEBUG_RENDER
            Quaternion rot = Quaternion.LookRotation(Vector3.forward, Vector3.up);
            debugMesh = RenderUtils.CreateQuadMesh(NodeVisualDebugSize, NodeVisualDebugSize, rot);
            movementRangeBatcher = new Batcher(debugMesh, debugMaterial);
            pathfindBatcher = new Batcher(debugMesh, debugMaterial);
#endif
        }

#if DEBUG_RENDER

        private void Update()
        {
            movementRangeBatcher.DoRender();
            pathfindBatcher.DoRender();
        }

#endif

        #endregion

        #region Pathfinding Methods

        /// <summary>
        /// Finds a path from the start node index to the end node index position.
        /// </summary>
        /// <param name="startNodeIndex"></param>
        /// <param name="endNodeIndex"></param>
        /// <param name="pathNodesIndicesResult"></param>
        public void FindNodeIndexPath(int startNodeIndex, int endNodeIndex, NativeList<int> pathNodesIndicesResult)
        {
            GridMaster gm = GridMaster.Instance;

            if (startNodeIndex < 0 || endNodeIndex < 0 || !gm.IsGridCreated)
            {
                Logger.LogWarning("Trying to find a path, either from a invalid start position or invalid end position, or maybe the grid was not created yet");
                return;
            }

            int dimension = gm.GridDepth * gm.GridWidth;

            FindPathJob findPathJob = new FindPathJob(dimension, gm.GridWidth, GridMaster.NodeNeighbors, startNodeIndex, endNodeIndex, gm.NodesNeighbors, gm.NodesTypes);
            JobHandle handle = findPathJob.Schedule();

            handle.Complete();

#if DEBUG_RENDER
            DebugPathfindingJob(ref findPathJob);
#endif

            // the path is given out from end to start, reverse it and fill the actual result list
            NativeList<int> reversedPath = findPathJob.pathResultNodesIndices;
            pathNodesIndicesResult.Clear();

            for (int i = reversedPath.Length - 1; i >= 0; i--)
                pathNodesIndicesResult.Add(reversedPath[i]);

            findPathJob.Dispose();
        }

        /// <summary>
        /// Finds a path from the start position to the end position.
        /// </summary>
        /// <param name="start"></param>
        /// <param name="end"></param>
        /// <param name="pathNodesIndicesResult"></param>
        public void FindNodeIndexPath(Vector3 start, Vector3 end, NativeList<int> pathNodesIndicesResult)
        {
            GridMaster gm = GridMaster.Instance;

            int startNodeIndex = gm.PosToNodeIndex(start);
            int endNodeIndex = gm.PosToNodeIndex(end);

            FindNodeIndexPath(startNodeIndex, endNodeIndex, pathNodesIndicesResult);
        }

        /// <summary>
        /// Finds a path from the start position to the end position.
        /// </summary>
        /// <param name="start"></param>
        /// <param name="end"></param>
        /// <param name="pathNodesTransformsResult"></param>
        /// <param name="positionYOffset"></param>
        public void FindNodePositionPath(Vector3 start, Vector3 end, NativeList<Vector3> pathNodesTransformsResult, float positionYOffset = 0.0f)
        {
            pathNodesTransformsResult.Clear();
            NativeList<int> nodeIndicesPath = new NativeList<int>(128, Allocator.TempJob);

            FindNodeIndexPath(start, end, nodeIndicesPath);

            GridMaster gm = GridMaster.Instance;

            // convert indices to the actual path based on points
            for (int i = 0; i < nodeIndicesPath.Length; i++)
            {
                int index = nodeIndicesPath[i];

                NodeTransform nt = gm.GetNodeTransform(index);
                Vector3 pos = nt.Pos;
                pos += nt.Up * positionYOffset;

                pathNodesTransformsResult.Add(pos);
            }

            nodeIndicesPath.Dispose();
        }

        /// <summary>
        /// Debugs the pathfinding algorithm from the job.
        /// </summary>
        /// <param name="findPathJob"></param>
        private void DebugPathfindingJob(ref FindPathJob findPathJob)
        {
#if DEBUG_RENDER
            if (debugPathfinding)
            {
                ClearPathfindingDebug();
                NativeArray<NodeTransform> nts = GridMaster.Instance.NodesTransforms;

                for (int j = 0; j < findPathJob.openSet.Length; j++)
                {
                    NodeTransform nt = nts[findPathJob.openSet[j]];
                    Matrix4x4 trs = Matrix4x4.TRS(nt.Pos + nt.Up * SmallUpDebugOffsetOpenClosedSet, nt.GetRotation(), Vector3.one);
                    pathfindBatcher.AddItem(openSetColor, trs);
                }

                for (int j = 0; j < findPathJob.closedSet.Length; j++)
                {
                    if (findPathJob.closedSet[j] != 1)
                        continue;

                    NodeTransform nt = nts[j];
                    Matrix4x4 trs = Matrix4x4.TRS(nt.Pos + nt.Up * SmallUpDebugOffsetOpenClosedSet, nt.GetRotation(), Vector3.one);
                    pathfindBatcher.AddItem(closedSetColor, trs);
                }

                for (int j = 0; j < findPathJob.pathResultNodesIndices.Length; j++)
                {
                    NodeTransform nt = nts[findPathJob.pathResultNodesIndices[j]];
                    Matrix4x4 trs = Matrix4x4.TRS(nt.Pos + nt.Up * SmallUpDebugOffsetPath, nt.GetRotation(), Vector3.one * ScalePathNode);
                    pathfindBatcher.AddItem(pathColor, trs);
                }
            }
#endif
        }

        #endregion

        #region Movement Range Methods

        /// <summary>
        /// Requests the range for a given character and stores the result in reachableNodes.
        /// </summary>
        /// <param name="fromPos"></param>
        /// <param name="depth"></param>
        /// <param name="reachableNodesList"></param>
        /// <param name="reachableNodesHashmap"></param>
        public void RequestRange(Vector3 fromPos, int depth, NativeList<int> reachableNodesList, NativeHashMap<int, int> reachableNodesHashmap)
        {
            GridMaster gm = GridMaster.Instance;
            int startIndex = gm.PosToNodeIndex(fromPos);

            if (!gm.IsGridCreated || startIndex < 0)
                return;

            int dimension = gm.GridDepth * gm.GridWidth;

            FindReachableNodesJob findReachableNodesJob = new FindReachableNodesJob(dimension, GridMaster.NodeNeighbors, startIndex, depth, gm.NodesTypes, gm.NodesNeighbors, reachableNodesList, reachableNodesHashmap);

            JobHandle jobHandle = findReachableNodesJob.Schedule();
            jobHandle.Complete();

#if DEBUG_RENDER
            if (debugMovementRange)
            {
                ClearMovementRangeDebug();
                NativeArray<NodeTransform> nts = gm.NodesTransforms;

                for (int j = 0; j < findReachableNodesJob.reachableNodesList.Length; j++)
                {
                    NodeTransform nt = nts[findReachableNodesJob.reachableNodesList[j]];
                    Matrix4x4 trs = Matrix4x4.TRS(nt.Pos + nt.Up * SmallUpDebugOffsetMovementRange, nt.GetRotation(), Vector3.one * ScaleMovementRangeNode);
                    movementRangeBatcher.AddItem(movementRangeDebugColor, trs);
                }
            }
#endif
            findReachableNodesJob.Dispose();
        }

        #endregion

        #region Debug Clearing

        /// <summary>
        /// Clears the debug for pathfinding.
        /// </summary>
        public void ClearPathfindingDebug()
        {
#if DEBUG_RENDER
            pathfindBatcher.Clear();
#endif
        }

        /// <summary>
        /// Clears the debug for the movement range.
        /// </summary>
        public void ClearMovementRangeDebug()
        {
#if DEBUG_RENDER
            movementRangeBatcher.Clear();
#endif
        }

        /// <summary>
        /// Clears all the debug.
        /// </summary>
        public void ClearAllDebug()
        {
            ClearMovementRangeDebug();
            ClearPathfindingDebug();
        }

        #endregion
    }
}