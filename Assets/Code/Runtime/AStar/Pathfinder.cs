using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;
using Logger = UnityLibrary.Logger;

namespace AStar
{
    /// <summary>
    /// Class to do pathfinding tasks using the nodes information from the GridMaster.
    /// </summary>
    public static class Pathfinder
    {
        #region Events

        public delegate void OnPathRetrievedDelegate(NativeList<int> openSet, NativeBitArray closedSet, NativeList<int> pathResultIndices);
        public static event OnPathRetrievedDelegate OnPathRetrieved;

        #endregion

        #region Pathfinding Jobs

        /// <summary>
        /// The job in charge of finding the path.
        /// </summary>
        [BurstCompile]
        private struct FindPathJob : IJob
        {
            public NativeList<int> openSet;
            public NativeBitArray closedSet;

            [WriteOnly] public NativeList<int> pathResultIndices;

            [ReadOnly] public NativeArray<NodeNeighbor> nodesNeighbors;
            [ReadOnly] public NativeArray<NodeType> nodesTypes;

            public int numNodes;
            public int gridWidth;
            public int numNeighbors;

            public int startNodeIndex;
            public int endNodeIndex;

            /// <inheritdoc/>
            public void Execute()
            {
                pathResultIndices.Clear();

                // Note: TODO: I have been reading lately and discovered in one side project, that
                // Temp allocator can fall back to an allocation that is even slower than TempJob. I
                // have no idea what is the size of the memory block that is used for Temp
                // allocations, which is reused, automatically "disposed" by Unity at the end of the
                // frame, and the reason why Temp allocations can not live more than 1 frame. If I
                // used this job in a massive way, I should look back at this, as I am completely
                // sure that the nodesInfo array is already a risk, easily occupying 3*4*10k bytes.
                NativeArray<NodePathFindInfo> nodesInfo = new NativeArray<NodePathFindInfo>(numNodes, Allocator.Temp);
                NativeBitArray openSetContains = new NativeBitArray(numNodes, Allocator.Temp);

                // set the info for the first node
                nodesInfo[startNodeIndex] = new NodePathFindInfo(0, GetHeuristic(startNodeIndex, endNodeIndex), -1);
                openSet.AddNoResize(startNodeIndex);

                while (openSet.Length > 0)
                {
                    int currNodeIndex = PopLowestFCostNodeIndexFromOpenSet(nodesInfo);

                    // we've reached the goal
                    if (currNodeIndex == endNodeIndex)
                    {
                        ReconstructPath(nodesInfo);
                        return;
                    }

                    // add it to the closed set by setting a flag at its index
                    closedSet.SetBits(currNodeIndex, true, 1);
                    NodePathFindInfo currNodeInfo = nodesInfo[currNodeIndex];

                    // go over the neighbors
                    int start = currNodeIndex * numNeighbors;
                    int end = start + numNeighbors;

                    for (int i = start; i < end; i++)
                    {
                        int neighborIndex = nodesNeighbors[i].neighborIndex;

                        // if it does not have neighbor, was already expanded or can't be walked by
                        if (!nodesNeighbors[i].isValid || closedSet.IsSet(neighborIndex) || (byte)nodesTypes[neighborIndex] > 0)
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
                            openSetContains.SetBits(neighborIndex, 1);
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

                //// Note: TODO? I think the only way to get here is if the way to a valid end node is
                //// completely blocked, in which case I should decide what to do
                //ReconstructPath(reversedPathResultIndices, nodesInfo);
                return;
            }

            /// <summary>
            /// Pops the lowest FCost node index from the open set.
            /// </summary>
            /// <param name="nodesInfo"></param>
            /// <returns></returns>
            private int PopLowestFCostNodeIndexFromOpenSet(NativeArray<NodePathFindInfo> nodesInfo)
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
            /// Gets the hCost from a node to another one.
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
            /// Reconstructs the path from the end index.
            /// </summary>
            /// <param name="nodesInfo"></param>
            private void ReconstructPath(NativeArray<NodePathFindInfo> nodesInfo)
            {
                FixedListInt4096 reversedPathResultIndices = new FixedListInt4096();

                int currNode = endNodeIndex;
                reversedPathResultIndices.AddNoResize(currNode);

                while (currNode != startNodeIndex)
                {
                    int parentNodeIndex = nodesInfo[currNode].parentNodeIndex;
                    currNode = parentNodeIndex;

                    reversedPathResultIndices.AddNoResize(parentNodeIndex);
                }

                // the path is given out from end to start, reverse it and fill the actual result list
                for (int i = reversedPathResultIndices.Length - 1; i >= 0; i--)
                    pathResultIndices.Add(reversedPathResultIndices[i]);
            }
        }

        #endregion

        #region Utility Jobs

        /// <summary>
        /// Job that converts the given node indices to the path positions with a configurable local
        /// normal offset.
        /// </summary>
        [BurstCompile]
        private struct ConvertNodeIndicesPathToPositionPath : IJob
        {
            [WriteOnly] public NativeList<Vector3> pathNodesPositionsResult;

            [ReadOnly] public NativeList<int> indices;
            [ReadOnly] public NativeArray<NodeTransform> nodeTransforms;

            public float normalOffset;

            /// <inheritdoc/>
            public void Execute()
            {
                for (int i = 0; i < indices.Length; i++)
                {
                    int index = indices[i];

                    NodeTransform nt = nodeTransforms[index];
                    pathNodesPositionsResult.Add((Vector3)(nt.pos + nt.up * normalOffset));
                }
            }
        }

        #endregion

        #region Pathfinding Methods

        /// <summary>
        /// Finds a path from the start node index to the end node index position.
        /// </summary>
        /// <param name="startNodeIndex"></param>
        /// <param name="endNodeIndex"></param>
        /// <param name="pathNodesIndicesResult"></param>
        /// <param name="forceRunOnMainThreadAvoidSchedule"></param>
        public static void FindNodeIndexPath(int startNodeIndex, int endNodeIndex, NativeList<int> pathNodesIndicesResult, bool forceRunOnMainThreadAvoidSchedule = false)
        {
            GridMaster gm = GridMaster.Instance;

            if (startNodeIndex < 0 || endNodeIndex < 0 || !gm.IsGridCreated)
            {
                Logger.LogWarning("Trying to find a path, either from a invalid start position or invalid end position, or maybe the grid was not created yet");
                return;
            }

            // Note: these could be made Temp inside the job but I need them so I can add visual debug
            int openSetFixedMaxElements = Mathf.Max(128, gm.Dimension / 2);
            NativeList<int> openSet = new NativeList<int>(openSetFixedMaxElements, Allocator.TempJob);
            NativeBitArray closedSet = new NativeBitArray(gm.Dimension, Allocator.TempJob);

            FindPathJob findPathJob = new FindPathJob
            {
                openSet = openSet,
                closedSet = closedSet,
                pathResultIndices = pathNodesIndicesResult,
                nodesNeighbors = gm.NodesNeighbors,
                nodesTypes = gm.NodesTypes,
                numNodes = gm.Dimension,
                gridWidth = gm.GridWidth,
                numNeighbors = GridMaster.NodeNumNeighbors,
                startNodeIndex = startNodeIndex,
                endNodeIndex = endNodeIndex,
            };

            // TODO: this schedule does not make any sense, I would have to implement a way to find paths "in batch" so that I actually take advantage of multithreading.
            // I also need a way to be able to schedule both pathfinding and range request combined,
            // since there is at least one situation where I need it.
            if (!forceRunOnMainThreadAvoidSchedule)
                findPathJob.Schedule().Complete();
            else
                findPathJob.Run();

            OnPathRetrieved?.Invoke(openSet, closedSet, pathNodesIndicesResult);

            openSet.Dispose();
            closedSet.Dispose();
        }

        /// <summary>
        /// Finds a path from the start position to the end position.
        /// </summary>
        /// <param name="start"></param>
        /// <param name="end"></param>
        /// <param name="pathNodesIndicesResult"></param>
        /// <param name="forceRunOnMainThreadAvoidSchedule"></param>
        public static void FindNodeIndexPath(Vector3 start, Vector3 end, NativeList<int> pathNodesIndicesResult, bool forceRunOnMainThreadAvoidSchedule = false)
        {
            GridMaster gm = GridMaster.Instance;

            int startNodeIndex = gm.PosToNodeIndex(start);
            int endNodeIndex = gm.PosToNodeIndex(end);

            FindNodeIndexPath(startNodeIndex, endNodeIndex, pathNodesIndicesResult, forceRunOnMainThreadAvoidSchedule);
        }

        /// <summary>
        /// Finds a path from the start position to the end position.
        /// </summary>
        /// <param name="start"></param>
        /// <param name="end"></param>
        /// <param name="pathNodesPositionsResult"></param>
        /// <param name="heightOffsetAlongNodeNormal"></param>
        /// <param name="forceRunOnMainThreadAvoidSchedule"></param>
        public static void FindNodePositionPath(Vector3 start, Vector3 end, NativeList<Vector3> pathNodesPositionsResult, float heightOffsetAlongNodeNormal = 0.0f, bool forceRunOnMainThreadAvoidSchedule = false)
        {
            pathNodesPositionsResult.Clear();

            NativeList<int> indices = new NativeList<int>(128, Allocator.TempJob);
            FindNodeIndexPath(start, end, indices, forceRunOnMainThreadAvoidSchedule);

            GridMaster gm = GridMaster.Instance;

            new ConvertNodeIndicesPathToPositionPath
            {
                pathNodesPositionsResult = pathNodesPositionsResult,
                indices = indices,
                nodeTransforms = gm.NodesTransforms,
                normalOffset = heightOffsetAlongNodeNormal,
            }
            .Run();

            indices.Dispose();
        }

        #endregion
    }
}