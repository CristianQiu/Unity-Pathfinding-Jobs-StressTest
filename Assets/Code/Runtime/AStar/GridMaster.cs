﻿using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;
using UnityLibrary;
using Logger = UnityLibrary.Logger;

namespace AStar
{
    /// <summary>
    /// The class in charge of scanning and building the level, constructing the nodes for the A*
    /// pathfinding system. Currently it scans a "2D" area in 3D space, which means no overlapping
    /// areas are allowed.
    /// </summary>
    public class GridMaster : MonoBehaviourSingleton<GridMaster>
    {
        #region Definitions

        /// <summary>
        /// Structure wrapping the needed settings to scan the area searching for colliders.
        /// </summary>
        private struct ScanAreaSettings
        {
            public readonly int gridWidth;
            public readonly int gridDepth;
            public readonly int gridDimension;
            public readonly float3 center;
            public readonly float3 extents;
            public readonly float3 flooredExtents;
            public readonly LayerMask mask;

            public ScanAreaSettings(float3 center, float3 extents, LayerMask mask)
            {
                this.center = center;
                this.extents = extents;
                this.mask = mask;

                // TODO: Don't allow negative extents when editing the collider
                float3 rest = extents % NodeSize;
                flooredExtents = extents - rest;

                gridWidth = (int)(flooredExtents.x / NodeSize) * 2;
                gridDepth = (int)(flooredExtents.z / NodeSize) * 2;
                gridDimension = gridWidth * gridDepth;
            }
        }

        #endregion

        #region Grid Creation Jobs

        /// <summary>
        /// Custom job to parallelize the calculation of launch points and directions for raycasts.
        /// </summary>
        [BurstCompile]
        private struct CalculateRaycastCommandsJob : IJobFor
        {
            [WriteOnly, NativeDisableParallelForRestriction] public NativeArray<RaycastCommand> commands;
            public ScanAreaSettings scanSettings;

            /// <inheritdoc/>
            public void Execute(int index)
            {
                // Note: I may not need this if all non primitive colliders are convex. We launch
                // five raycasts per node so that we are able to handle realistic stairs. The rays
                // are set in the following order: middle point, upper right, lower right, lower
                // left, upper left (that is clockwise and in world space)
                int row = index / scanSettings.gridWidth;
                int col = index % scanSettings.gridWidth;

                float midX = scanSettings.center.x - scanSettings.flooredExtents.x + (col * NodeSize) + NodeHalfSize;
                float midZ = scanSettings.center.z - scanSettings.flooredExtents.z + (row * NodeSize) + NodeHalfSize;
                float y = scanSettings.center.y + scanSettings.extents.y;

                float3 midRayStartPos = new float3(midX, y, midZ);
                float3 toUpperRightOffset = (math.forward() + math.right()) * (NodeHalfSize - 0.05f);
                float3 toLowerRightOffset = (math.back() + math.right()) * (NodeHalfSize - 0.05f);

                float3 upperRight = midRayStartPos + toUpperRightOffset;
                float3 lowerRight = midRayStartPos + toLowerRightOffset;
                float3 lowerLeft = midRayStartPos - toUpperRightOffset;
                float3 upperLeft = midRayStartPos - toLowerRightOffset;

                float rayDist = scanSettings.extents.y * 2.0f;
                Vector3 down = Vector3.down;

                commands[index * 5] = new RaycastCommand((Vector3)midRayStartPos, down, rayDist, scanSettings.mask, 1);
                commands[index * 5 + 1] = new RaycastCommand((Vector3)upperRight, down, rayDist, scanSettings.mask, 1);
                commands[index * 5 + 2] = new RaycastCommand((Vector3)lowerRight, down, rayDist, scanSettings.mask, 1);
                commands[index * 5 + 3] = new RaycastCommand((Vector3)lowerLeft, down, rayDist, scanSettings.mask, 1);
                commands[index * 5 + 4] = new RaycastCommand((Vector3)upperLeft, down, rayDist, scanSettings.mask, 1);
            }
        }

        /// <summary>
        /// The job to create the nodes transforms using the hits position, or if invalid, the
        /// original XZ position of the ray launched.
        /// </summary>
        [BurstCompile]
        private struct CreateNodesJob : IJobFor
        {
            [WriteOnly] public NativeArray<NodeTransform> nodesTransforms;
            [WriteOnly] public NativeArray<NodeType> nodesTypes;

            [ReadOnly] public NativeArray<RaycastHit> hits;
            [ReadOnly] public NativeArray<RaycastCommand> commands;

            /// <inheritdoc/>
            public void Execute(int index)
            {
                RaycastHit midHit = hits[index * 5];
                RaycastCommand midCommand = commands[index * 5];

                // we can't check for collider to be null since reference types are not allowed
                bool validNode = midHit.normal != default(Vector3);

                float3 pos;
                float3 normal;

                if (validNode)
                {
                    bool isStair = IsValidIndexNodeStair(index, out float3 fakeStairNormal);

                    normal = math.select((float3)midHit.normal, fakeStairNormal, isStair);
                    pos = (float3)midHit.point;
                }
                else
                {
                    pos = new float3(midCommand.from.x, 0.0f, midCommand.from.z);
                    normal = math.up();
                }

                nodesTransforms[index] = new NodeTransform(pos, normal);
                nodesTypes[index] = !validNode ? NodeType.Invalid : NodeType.Free;
            }

            /// <summary>
            /// Get whether the valid node index is considered as a stair, and if so return the
            /// computed "fake" normal, which would belong to the ramp formed by joining the four
            /// corners of the node.
            /// </summary>
            /// <param name="index"></param>
            /// <param name="computedFakeNormal"></param>
            /// <returns></returns>
            private bool IsValidIndexNodeStair(int index, out float3 computedFakeNormal)
            {
                float3 mid = (float3)hits[index * 5].point;
                float3 upperRight = (float3)hits[index * 5 + 1].point;
                float3 lowerRight = (float3)hits[index * 5 + 2].point;
                float3 lowerLeft = (float3)hits[index * 5 + 3].point;
                float3 upperLeft = (float3)hits[index * 5 + 4].point;

                bool isStair = true;
                computedFakeNormal = float3.zero;

                for (int i = index * 5; i <= index * 5 + 4; i++)
                {
                    RaycastHit hit = hits[i];

                    float dot = math.abs(math.dot((float3)hit.normal, math.up()));
                    float heightDist = math.abs(mid.y - hit.point.y);

                    if (dot <= (1.0f - MinDotErrorToConsiderRamp) || heightDist < MinHeightDistToConsiderStepInSameNode && (i != index * 5))
                    {
                        isStair = false;
                        break;
                    }
                }

                if (isStair)
                {
                    float3 upperLeftToUpperRight = math.normalize(upperRight - upperLeft);
                    float3 lowerLeftToLowerRight = math.normalize(lowerRight - lowerLeft);

                    // this is not really necessary but in the case of minimal errors with the
                    // geometry it may smooth the result
                    float3 leftToRight = math.normalize(upperLeftToUpperRight + lowerLeftToLowerRight);

                    float3 lowerLeftToUpperLeft = math.normalize(upperLeft - lowerLeft);
                    float3 lowerRightToUpperRight = math.normalize(upperRight - lowerRight);

                    // same as before, not strictly required
                    float3 lowerToUpper = math.normalize(lowerLeftToUpperLeft + lowerRightToUpperRight);

                    computedFakeNormal = math.normalize(math.cross(lowerToUpper, leftToRight));
                }

                return isStair;
            }
        }

        #endregion

        #region Bake Obstacles Jobs

        /// <summary>
        /// The job to prepare the boxcasting commands to launch them to recognize obstacles in the grid.
        /// </summary>
        [BurstCompile]
        private struct CalculateBoxcastCommandsJob : IJobFor
        {
            [WriteOnly] public NativeArray<BoxcastCommand> commands;
            [ReadOnly] public NativeArray<NodeTransform> nodesTransforms;

            public LayerMask mask;
            public float boxNodePercentage;
            public float maxCharacterHeight;

            /// <inheritdoc/>
            public void Execute(int index)
            {
                NodeTransform nt = nodesTransforms[index];

                // start a bit before the node just in case there's an obstacle overlapping a bit
                Vector3 center = (Vector3)(nt.pos - nt.up * 0.1f);

                // nodes are squares and we don't plan to change it
                float halfWidth = NodeHalfSize * boxNodePercentage;
                float halfDepth = halfWidth;
                Vector3 halfExtents = new Vector3(halfWidth, 0.01f, halfDepth);

                commands[index] = new BoxcastCommand(center, halfExtents, (Quaternion)nt.GetRotation(), (Vector3)nt.up, maxCharacterHeight, mask);
            }
        }

        /// <summary>
        /// The job that takes the results from the boxcasts and makes the node walkable or not
        /// depending on the result.
        /// </summary>
        [BurstCompile]
        private struct BakeObstaclesJob : IJobFor
        {
            public NativeArray<NodeType> nodesTypes;
            [ReadOnly] public NativeArray<RaycastHit> boxcastHits;

            /// <inheritdoc/>
            public void Execute(int index)
            {
                RaycastHit hit = boxcastHits[index];
                NodeType nodeType = nodesTypes[index];

                // if the node was not valid, we won't modify its state even if occupied by an obstacle
                NodeType newNodeType = hit.normal == default(Vector3) ? NodeType.Free : NodeType.OccupiedByObstacle;
                newNodeType = nodeType == NodeType.Invalid ? NodeType.Invalid : newNodeType;

                nodesTypes[index] = newNodeType;
            }
        }

        #endregion

        #region Grid Neighbors Jobs

        /// <summary>
        /// The job that calculates the neighbors for each node.
        /// </summary>
        [BurstCompile]
        private struct CalculateNeighborsJob : IJobFor
        {
            [WriteOnly, NativeDisableParallelForRestriction] public NativeArray<NodeNeighbor> neighbors;

            [ReadOnly] public NativeArray<NodeTransform> nodesTransforms;

            public ScanAreaSettings scanSettings;
            public float maxWalkableHeightWithStep;

            /// <inheritdoc/>
            public void Execute(int index)
            {
                FixedListInt32 neighborIndices = new FixedListInt32();
                int nodeRow = index / scanSettings.gridWidth;

                int topIndex = index + scanSettings.gridWidth;
                int rightIndex = (index + 1) / scanSettings.gridWidth != nodeRow ? -1 : index + 1;
                int botIndex = index - scanSettings.gridWidth;
                int leftIndex = (index - 1) / scanSettings.gridWidth != nodeRow ? -1 : index - 1;

                // the order is important, its used by some of the pathfinder algorithms
                neighborIndices.AddNoResize(topIndex);
                neighborIndices.AddNoResize(rightIndex);
                neighborIndices.AddNoResize(botIndex);
                neighborIndices.AddNoResize(leftIndex);

                int numNeighbors = NodeNumNeighbors;

                for (int i = 0; i < numNeighbors; i++)
                {
                    int neighborIndex = neighborIndices[i];
                    int neighborsArrayIndex = index * numNeighbors + i;

                    if (neighborIndex < 0 || neighborIndex >= nodesTransforms.Length)
                    {
                        // out of bounds
                        neighbors[neighborsArrayIndex] = new NodeNeighbor(-1, false);
                        continue;
                    }

                    bool canReachNeighbor = CanReachNeighbor(index, neighborIndex);
                    neighbors[neighborsArrayIndex] = new NodeNeighbor(neighborIndex, canReachNeighbor);
                }
            }

            /// <summary>
            /// Get whether it is possible to reach a given neighbor from the given node index.
            /// </summary>
            /// <param name="index"></param>
            /// <param name="neighborIndex"></param>
            /// <returns></returns>
            private bool CanReachNeighbor(int index, int neighborIndex)
            {
                NodeTransform nt = nodesTransforms[index];
                NodeTransform ntn = nodesTransforms[neighborIndex];

                // the following code will "cut corners", so the path to go over ramps works as intended
                bool canReachNeighbor = false;

                int rowNode = index / scanSettings.gridWidth;
                int rowNeighbor = neighborIndex / scanSettings.gridWidth;

                bool sameRow = rowNode == rowNeighbor;
                bool sameCol = (index + scanSettings.gridWidth == neighborIndex) || (index - scanSettings.gridWidth == neighborIndex);

                const float dotThreshold = 0.99625f; // anything over is aprox 5 degrees or less in "angle distance"
                float dotRightsAbs = math.abs(math.dot(nt.right, ntn.right));
                float dotFwdsAbs = math.abs(math.dot(nt.fwd, ntn.fwd));

                if ((sameCol && dotRightsAbs >= dotThreshold) || (sameRow && dotFwdsAbs >= dotThreshold))
                {
                    // the node can be reached if the distance in height meets the requirements
                    canReachNeighbor = math.distance(nt.pos.y, ntn.pos.y) <= maxWalkableHeightWithStep;
                }

                return canReachNeighbor;
            }
        }

        #endregion

        #region Events

        public delegate void OnGridCreationDelegate();
        public event OnGridCreationDelegate OnGridCreation;

        #endregion

        #region Public Attributes

        public const float NodeSize = 1.0f;
        public const float NodeHalfSize = NodeSize * 0.5f;
        public const int NodeNumNeighbors = (int)NeighborLayout.Four;

        #endregion

        #region Private Attributes

        private const float MinDotErrorToConsiderRamp = 0.01f;
        private const float MinHeightDistToConsiderStepInSameNode = 0.01f;

        [Header("Construction settings")]
        [SerializeField] private LayerMask walkableMask = default;
        [SerializeField] private LayerMask obstacleMask = default;

        [SerializeField, Range(0.0f, 5.0f)] private float maxCharacterHeight = 2.0f;
        [SerializeField, Range(0.0f, 1.0f)] private float boxToNodeObstaclePercentage = 0.90f;
        [SerializeField, Range(0.0f, 1.0f)] private float maxWalkableHeightWithStep = 0.25f;

        private BoxCollider scanCollider;
        private bool isGridCreated;
        private int gridWidth;
        private int gridDepth;

        private NativeArray<NodeTransform> nodesTransforms;
        private NativeArray<NodeType> nodesTypes;
        private NativeArray<NodeNeighbor> nodesNeighbors;

        #endregion

        #region Properties

        protected override bool DestroyOnLoad { get { return true; } }

        public bool IsGridCreated { get { return isGridCreated; } }
        public int GridWidth { get { return gridWidth; } }
        public int GridDepth { get { return gridDepth; } }
        public int Dimension { get { return gridWidth * gridDepth; } }
        public Bounds Bounds { get { return scanCollider.bounds; } }
        public NativeArray<NodeTransform> NodesTransforms { get { return nodesTransforms; } }
        public NativeArray<NodeType> NodesTypes { get { return nodesTypes; } }
        public NativeArray<NodeNeighbor> NodesNeighbors { get { return nodesNeighbors; } }

        #endregion

        #region MonoBehaviour Methods

        protected override void Awake()
        {
            base.Awake();
            scanCollider = GetComponent<BoxCollider>();
        }

        private void OnDestroy()
        {
            DestroyGrid();
        }

        #endregion

        #region Initialization Methods

        /// <summary>
        /// Creates the grid of nodes.
        /// </summary>
        public void CreateGrid()
        {
            DestroyGrid();

            // TODO: Perhaps we might want to snap the extents value when editing the bounding box
            // in the editor?
            Bounds scanBounds = scanCollider.bounds;
            ScanAreaSettings scanSettings = new ScanAreaSettings((float3)scanBounds.center, (float3)scanBounds.extents, walkableMask);
            int expectedGridDimension = scanSettings.gridDimension;

            // TODO: Could I use nodesTypes invalid to avoid any kind of computation from them?
            // TODO: Could I actually initialize it without clearing memory?
            nodesTransforms = new NativeArray<NodeTransform>(expectedGridDimension, Allocator.Persistent);
            nodesTypes = new NativeArray<NodeType>(expectedGridDimension, Allocator.Persistent);
            nodesNeighbors = new NativeArray<NodeNeighbor>(expectedGridDimension * NodeNumNeighbors, Allocator.Persistent);

            // calculate the initial raycast commands
            NativeArray<RaycastCommand> mainCommands = new NativeArray<RaycastCommand>(expectedGridDimension * 5, Allocator.TempJob);

            JobHandle createNodesHandle = new CalculateRaycastCommandsJob
            {
                commands = mainCommands,
                scanSettings = scanSettings,
            }
            .ScheduleParallel(expectedGridDimension, 64, default(JobHandle));

            // schedule the commands to retrieve the initial hits
            NativeArray<RaycastHit> nodeHits = new NativeArray<RaycastHit>(expectedGridDimension * 5, Allocator.TempJob);
            createNodesHandle = RaycastCommand.ScheduleBatch(mainCommands, nodeHits, 32, createNodesHandle);

            // build the nodes using the received hits and the main raycast commands
            createNodesHandle = new CreateNodesJob
            {
                nodesTransforms = nodesTransforms,
                nodesTypes = nodesTypes,
                hits = nodeHits,
                commands = mainCommands,
            }
            .ScheduleParallel(expectedGridDimension, 32, createNodesHandle);

            // calculate the boxcasts to bake obstacles
            NativeArray<BoxcastCommand> boxcastCommands = new NativeArray<BoxcastCommand>(expectedGridDimension, Allocator.TempJob);

            JobHandle bakeObstaclesHandle = new CalculateBoxcastCommandsJob
            {
                commands = boxcastCommands,
                nodesTransforms = nodesTransforms,
                mask = obstacleMask,
                boxNodePercentage = boxToNodeObstaclePercentage,
                maxCharacterHeight = maxCharacterHeight,
            }
            .ScheduleParallel(expectedGridDimension, 64, createNodesHandle);

            // schedule the boxcasts to find possible obstacles
            NativeArray<RaycastHit> obstacleHits = new NativeArray<RaycastHit>(expectedGridDimension, Allocator.TempJob);
            bakeObstaclesHandle = BoxcastCommand.ScheduleBatch(boxcastCommands, obstacleHits, 32, bakeObstaclesHandle);

            // prepare the bake obstacles job
            bakeObstaclesHandle = new BakeObstaclesJob
            {
                nodesTypes = nodesTypes,
                boxcastHits = obstacleHits,
            }
            .ScheduleParallel(expectedGridDimension, 128, bakeObstaclesHandle);

            // now calculate the neighbors
            JobHandle calculateNeighborsHandle = new CalculateNeighborsJob
            {
                neighbors = nodesNeighbors,
                nodesTransforms = nodesTransforms,
                scanSettings = scanSettings,
                maxWalkableHeightWithStep = maxWalkableHeightWithStep,
            }
            .ScheduleParallel(expectedGridDimension, 32, createNodesHandle);

            JobHandle finalHandle = JobHandle.CombineDependencies(calculateNeighborsHandle, bakeObstaclesHandle);

            JobHandle disposeHandle = JobHandle.CombineDependencies(mainCommands.Dispose(finalHandle), nodeHits.Dispose(finalHandle));
            disposeHandle = JobHandle.CombineDependencies(disposeHandle, boxcastCommands.Dispose(finalHandle), obstacleHits.Dispose(finalHandle));

            // wait to complete all the scheduled stuff
            finalHandle.Complete();

            gridWidth = scanSettings.gridWidth;
            gridDepth = scanSettings.gridDepth;
            isGridCreated = true;

            OnGridCreation?.Invoke();

            Logger.LogFormat("Grid was created with dimension {0}. Width: {1}. Height: {2}.", expectedGridDimension, gridWidth, gridDepth);

            disposeHandle.Complete();
        }

        private void DestroyGrid()
        {
            if (isGridCreated)
            {
                isGridCreated = false;
                gridWidth = 0;
                gridDepth = 0;
                if (nodesTransforms.IsCreated)
                    nodesTransforms.Dispose();
                if (nodesTypes.IsCreated)
                    nodesTypes.Dispose();
                if (nodesNeighbors.IsCreated)
                    nodesNeighbors.Dispose();
            }
        }

        #endregion

        #region Methods

        /// <summary>
        /// Returns the closest point to the given position that is within the bounding box.
        /// </summary>
        /// <param name="pos"></param>
        /// <returns></returns>
        public Vector3 ClampPosToNodesBoundingBox(Vector3 pos)
        {
            return scanCollider.bounds.ClosestPoint(pos);
        }

        /// <summary>
        /// Converts a world position to a node index. Returns -1 if the position is not within the grid.
        /// </summary>
        /// <param name="pos"></param>
        /// <returns></returns>
        public int PosToNodeIndex(Vector3 pos)
        {
            int index = -1;

            Vector3 localPos = pos - scanCollider.bounds.center;
            Vector3 extents = scanCollider.bounds.extents;

            // TODO: Don't allow negative extents when editing the collider
            float restX = extents.x % NodeSize;
            float restZ = extents.z % NodeSize;

            float flooredX = extents.x - restX;
            float flooredZ = extents.z - restZ;

            Vector3 flooredExtents = new Vector3(flooredX, 0.0f, flooredZ);

            if (!isGridCreated ||
                localPos.x < -flooredExtents.x || localPos.x > flooredExtents.x ||
                localPos.z < -flooredExtents.z || localPos.z > flooredExtents.z)
                return index;

            // get rid of negative values
            localPos += flooredExtents;

            int row = Mathf.FloorToInt(localPos.z / NodeSize);
            int col = Mathf.FloorToInt(localPos.x / NodeSize);

            return row * gridWidth + col;
        }

        /// <summary>
        /// Get the transform of the node corresponding to a position.
        /// </summary>
        /// <param name="pos"></param>
        /// <returns></returns>
        public NodeTransform PosToNode(Vector3 pos)
        {
            int index = PosToNodeIndex(pos);

            if (index < 0)
            {
                Logger.LogWarning("Trying to convert a position to a node, but it is not possible because there is not a node that corresponds to the given position. Returning default value for the node.");
                return default(NodeTransform);
            }

            return nodesTransforms[index];
        }

        /// <summary>
        /// Gets the node transform associated to a node index.
        /// </summary>
        /// <param name="nodeIndex"></param>
        /// <returns></returns>
        public NodeTransform GetNodeTransform(int nodeIndex)
        {
            if (NodeIndexOutOfBounds(nodeIndex))
            {
                Logger.LogWarningFormat("Trying to get a node transform from invalid node index {0}", nodeIndex.ToString());
                return default(NodeTransform);
            }

            return nodesTransforms[nodeIndex];
        }

        /// <summary>
        /// Gets how is the given node index occupied state.
        /// </summary>
        /// <param name="nodeIndex"></param>
        /// <returns></returns>
        public NodeType GetNodeOccupation(int nodeIndex)
        {
            if (NodeIndexOutOfBounds(nodeIndex))
            {
                Logger.LogWarningFormat("Trying to get node occupation from invalid node index {0}", nodeIndex.ToString());
                return NodeType.Invalid;
            }

            return nodesTypes[nodeIndex];
        }

        /// <summary>
        /// Sets the node occupation at the given pos. It can also be freed from any character that
        /// may be occupying it.
        /// </summary>
        /// <param name="pos"></param>
        /// <param name="occupiedReason"></param>
        public void SetNodeOccupation(Vector3 pos, NodeType occupiedReason)
        {
            int index = PosToNodeIndex(pos);

            SetNodeOccupation(index, occupiedReason);
        }

        /// <summary>
        /// Sets the node occupation at the node index. It can also be freed from any character that
        /// may be occupying it.
        /// </summary>
        /// <param name="nodeIndex"></param>
        /// <param name="occupiedReason"></param>
        public void SetNodeOccupation(int nodeIndex, NodeType occupiedReason)
        {
            if (NodeIndexOutOfBounds(nodeIndex))
            {
                Logger.LogWarning("Trying to occupy a node, but it is not possible because the node index is invalid.");
                return;
            }

            nodesTypes[nodeIndex] = occupiedReason;
        }

        /// <summary>
        /// Gets whether the given node index is out of the bounds.
        /// </summary>
        /// <param name="nodeIndex"></param>
        /// <returns></returns>
        private bool NodeIndexOutOfBounds(int nodeIndex)
        {
            return nodeIndex < 0 || nodeIndex >= gridWidth * gridDepth;
        }

        #endregion
    }
}