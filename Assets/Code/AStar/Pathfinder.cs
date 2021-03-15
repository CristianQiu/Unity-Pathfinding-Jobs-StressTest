using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
using Unity.Mathematics;

namespace AStar
{
    /// <summary>
    /// Class to do pathfinding tasks using the nodes information from the GridMaster.
    /// </summary>
    public static class Pathfinder
    {
        #region Pathfinding Jobs

        /// <summary>
        /// The job in charge of finding the path.
        /// </summary>
        [BurstCompile]
        private struct FindPathJob : IJob
        {
            [WriteOnly, NativeDisableContainerSafetyRestriction] public NativeArray<int> nextIndices;

            [ReadOnly] public NativeArray<int> startIndices;
            [ReadOnly] public NativeArray<int> endIndices;
            [ReadOnly] public NativeArray<NodeNeighbor> nodesNeighbors;
            [ReadOnly] public NativeArray<NodeType> nodesTypes;

            public int jobIndex;

            public int numNodes;
            public int gridWidth;
            public int numNeighbors;

            private int startNodeIndex;
            private int endNodeIndex;

            /// <inheritdoc/>
            public void Execute()
            {
                startNodeIndex = startIndices[jobIndex];
                endNodeIndex = endIndices[jobIndex];

                // Note: Temp allocator can fall back to a very much slower version if the block of
                // memory that it uses is exhausted. By the looks of the tests that I have done, it
                // seems that this memory is released after the job is finished. I had this
                // originally in an IJobFor or IJobParallelFor and there were threads that
                // introduced significant bottlenecks due to this issue (the inner loop batch count
                // didn't matter), but after switching to a "batch of IJobs" this issue is gone, as
                // the maximum jobs that can run at the same time is just the number of logical
                // threads of the system, which shouldn't be more than ~32 in a high end system
                // (although some threadrippers can go up to 128, which eventually may be an issue
                // again). I have tested this in a complete flat map with no obstacles, and the time
                // to complete each job is reasonably similar.

                // I have tried swapping this by just a plain int3 and surprisingly, it is
                // substantially slower, I have also tried to go with ushorts rather than full ints,
                // and it does not make an identifiable difference.
                NativeArray<NodePathFindInfo> nodesInfo = new NativeArray<NodePathFindInfo>(numNodes, Allocator.Temp);
                NativeBitArray closedSet = new NativeBitArray(numNodes, Allocator.Temp);
                NativeBitArray openSetContains = new NativeBitArray(numNodes, Allocator.Temp);

                // Warning: 272 is a magical number due to the map layout and possible paths, which
                // seems to not throw errors about being too low for the test scene. This list
                // should be somewhat constrained to a low range in order to keep the algorithm
                // performant, otherwise it would be worth to look at implementing a native binary
                // heap to speed up the extaction of the lowest fcost node.
                NativeList<int> openSet = new NativeList<int>(272, Allocator.Temp);

                // set the info for the first node
                nodesInfo[startNodeIndex] = new NodePathFindInfo(0, GetHeuristic(startNodeIndex, endNodeIndex), -1);
                openSet.AddNoResize(startNodeIndex);

                while (openSet.Length > 0)
                {
                    int currNodeIndex = PopLowestFCostNodeIndexFromOpenSet(openSet, nodesInfo);

                    // we've reached the goal
                    if (currNodeIndex == endNodeIndex)
                    {
                        SaveNextNodeIndexToMoveTo(nodesInfo);
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

                // Note: TODO? I think the only way to get here is if the way to a valid end node is
                // completely blocked, in which case I should decide what to do
                return;
            }

            /// <summary>
            /// Pops the lowest FCost node index from the open set.
            /// </summary>
            /// <param name="openSet"></param>
            /// <param name="nodesInfo"></param>
            /// <returns></returns>
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

                // I have tested RemoveAt vs RemoveAtSwapBack, and oddly enough there seems to be a
                // extremely slightly difference in favour of RemoveAt, but perhaps this is due to
                // the non-totally-deterministic stress test... I'm still surprised there is not a
                // notable difference in favour of RemoveAtSwapBack though.
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
            /// Reconstructs the path from the end index, and saves the next node index to move to
            /// in the corresponding position of the array.
            /// </summary>
            /// <param name="nodesInfo"></param>
            private void SaveNextNodeIndexToMoveTo(NativeArray<NodePathFindInfo> nodesInfo)
            {
                int currNode = endNodeIndex;
                int lastParent = -1;

                while (currNode != startNodeIndex)
                {
                    int parentNodeIndex = nodesInfo[currNode].parentNodeIndex;
                    lastParent = currNode;
                    currNode = parentNodeIndex;
                }

                nextIndices[jobIndex] = lastParent;
            }
        }

        #endregion

        #region Pathfinding Methods

        /// <summary>
        /// Schedules one pathfinding job.
        /// </summary>
        /// <param name="gm"></param>
        /// <param name="startIndices"></param>
        /// <param name="endIndices"></param>
        /// <param name="nextIndices"></param>
        /// <param name="jobIndex"></param>
        /// <param name="deps"></param>
        /// <returns></returns>
        public static JobHandle ScheduleFindPath(GridMaster gm, NativeArray<int> startIndices, NativeArray<int> endIndices, NativeArray<int> nextIndices, int jobIndex, JobHandle deps)
        {
            return new FindPathJob
            {
                nextIndices = nextIndices,
                startIndices = startIndices,
                endIndices = endIndices,
                nodesNeighbors = gm.NodesNeighbors,
                nodesTypes = gm.NodesTypes,
                jobIndex = jobIndex,
                numNodes = gm.Dimension,
                gridWidth = gm.GridWidth,
                numNeighbors = GridMaster.NodeNumNeighbors,
            }
            .Schedule(deps);
        }

        #endregion
    }
}