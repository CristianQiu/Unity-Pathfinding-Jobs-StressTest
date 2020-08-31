using System.Collections.Generic;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;
using UnityLibrary;

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

            [DeallocateOnJobCompletion] private NativeArray<NodePathFindInfo> nodesInfo;

            public NativeList<int> openSet;

            // we need a one-element array to be able to read the result after jobsdone
            [WriteOnly] public NativeArray<int> nextNodeOnPath;

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

                nextNodeOnPath = new NativeArray<int>(1, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);
            }

            /// <inheritdoc/>
            public void Execute()
            {
                NativeArray<byte> closedSet = new NativeArray<byte>(numNodes, Allocator.Temp);

                // when an index is invalid, it's set to -1
                if (startNodeIndex < 0 || endNodeIndex < 0)
                {
                    nextNodeOnPath[0] = -1;
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
                        CalculateNextNodeIndexToMoveTo();
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

                nextNodeOnPath[0] = -1;
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
            /// Calculates the next node index that we should move to.
            /// </summary>
            private void CalculateNextNodeIndexToMoveTo()
            {
                int currNode = endNodeIndex;
                int nextNode = -1;

                while (currNode != startNodeIndex)
                {
                    nextNode = currNode;
                    int parentNodeIndex = nodesInfo[currNode].parentNodeIndex;
                    currNode = parentNodeIndex;
                }

                nextNodeOnPath[0] = nextNode;
            }

            /// <summary>
            /// Deallocates the datastructures used by this job.
            /// </summary>
            public void Dispose()
            {
                openSet.Dispose();
                //nodesInfo.Dispose();
                nextNodeOnPath.Dispose();
            }
        }

        #endregion

        #region Private Attributes

        private List<FindPathJob> findPathJobs = new List<FindPathJob>(1024);

        #endregion

        #region Properties

        protected override bool DestroyOnLoad { get { return true; } }

        #endregion

        #region Pathfinding Methods

        /// <summary> Schedules and completes a bunch of paths, as many as startPositions &
        /// endPositions length. Fills the nextNodesIndices array with the resulting next nodes to
        /// move to. </summary> <param name="startPositions"></param> <param
        /// name="endPositions"></param> <param name="nextNodesIndices"></param>
        public void FindNextNodeIndexOnPathsBatch(NativeArray<Vector3> startPositions, NativeArray<Vector3> endPositions, NativeArray<int> nextNodesIndices)
        {
            GridMaster gm = GridMaster.Instance;
            int dimension = gm.GridWidth * gm.GridDepth;

            int numPaths = startPositions.Length;

            NativeArray<JobHandle> handles = new NativeArray<JobHandle>(numPaths, Allocator.Temp, NativeArrayOptions.UninitializedMemory);

            for (int i = 0; i < numPaths; i++)
            {
                int startNodeIndex = gm.PosToNodeIndex(startPositions[i]);
                int endNodeIndex = gm.PosToNodeIndex(endPositions[i]);

                FindPathJob findPathJob = new FindPathJob(dimension, gm.GridWidth, GridMaster.NodeNeighbors, startNodeIndex, endNodeIndex, gm.NodesNeighbors, gm.NodesTypes);
                handles[i] = findPathJob.Schedule();

                findPathJobs.Add(findPathJob);
            }

            JobHandle.CompleteAll(handles);

            for (int i = findPathJobs.Count - 1; i >= 0; i--)
            {
                nextNodesIndices[i] = findPathJobs[i].nextNodeOnPath[0];
                findPathJobs[i].Dispose();
                findPathJobs.RemoveAt(i);
            }

            handles.Dispose();
        }

        #endregion
    }
}