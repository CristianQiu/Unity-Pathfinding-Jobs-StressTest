using System.Collections.Generic;
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
                openSet = new NativeList<int>(numNodes / 4, Allocator.TempJob);

                pathResultNodesIndices = new NativeList<int>(Allocator.TempJob);
            }

            /// <inheritdoc/>
            public void Execute()
            {
                NativeArray<byte> closedSet = new NativeArray<byte>(numNodes, Allocator.Temp);

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
                        closedSet.Dispose();
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
                closedSet.Dispose();
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
                nodesInfo.Dispose();
                pathResultNodesIndices.Dispose();
            }
        }

        #endregion

        #region Private Attributes

        private List<FindPathJob> findPathJobs = new List<FindPathJob>(1024);

        #endregion

        #region Properties

        protected override bool DestroyOnLoad { get { return true; } }

        #endregion

        #region MonoBehaviour Methods

        protected override void Awake()
        {
            base.Awake();
        }

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

        /// <summary> Schedules and completes a bunch of paths, as many as startPositions &
        /// endPositions length. </summary> <param name="startPositions"></param> <param name="endPositions"></param>
        public void FindNextPointOnPathsBatch(NativeArray<Vector3> startPositions, NativeArray<Vector3> endPositions)
        {
            GridMaster gm = GridMaster.Instance;
            int dimension = gm.GridWidth * gm.GridDepth;

            int numPaths = startPositions.Length;

            NativeArray<JobHandle> handles = new NativeArray<JobHandle>(numPaths, Allocator.Temp, NativeArrayOptions.UninitializedMemory);

            for (int i = 0; i < numPaths; i++)
            {
                int startNodeIndex = gm.PosToNodeIndex(startPositions[i]);
                int endNodeIndex = gm.PosToNodeIndex(endPositions[i]);

                //if (startNodeIndex == -1 || endNodeIndex == -1)
                //{
                //    Debug.Log("invalid i " + i);
                //}

                FindPathJob findPathJob = new FindPathJob(dimension, gm.GridWidth, GridMaster.NodeNeighbors, startNodeIndex, endNodeIndex, gm.NodesNeighbors, gm.NodesTypes);
                handles[i] = findPathJob.Schedule();

                findPathJobs.Add(findPathJob);
                //findPathJob.pathResultNodesIndices;
            }

            JobHandle.CompleteAll(handles);

            for (int i = findPathJobs.Count - 1; i >= 0; i--)
            {
                findPathJobs[i].Dispose();
                findPathJobs.RemoveAt(i);
            }

            handles.Dispose();
        }

        #endregion
    }
}