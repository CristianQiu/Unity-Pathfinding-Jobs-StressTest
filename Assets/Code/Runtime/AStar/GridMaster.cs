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
                int row = index / scanSettings.gridWidth;
                int col = index % scanSettings.gridWidth;

                float midX = scanSettings.center.x - scanSettings.flooredExtents.x + (col * NodeSize) + NodeHalfSize;
                float midZ = scanSettings.center.z - scanSettings.flooredExtents.z + (row * NodeSize) + NodeHalfSize;
                float y = scanSettings.center.y + scanSettings.extents.y;

                Vector3 startPos = new Vector3(midX, y, midZ);
                float rayDist = scanSettings.extents.y * 2.0f;

                commands[index] = new RaycastCommand(startPos, Vector3.down, rayDist, scanSettings.mask, 1);
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
                RaycastHit hit = hits[index];
                RaycastCommand command = commands[index];

                // we can't check for collider to be null since reference types are not allowed
                bool validNode = hit.normal != default(Vector3);

                float3 commandPos = command.from;
                commandPos.y = 0.0f;

                float3 pos = math.select(commandPos, (float3)hit.point, validNode);
                float3 normal = math.select(math.up(), (float3)hit.normal, validNode);

                nodesTransforms[index] = new NodeTransform(pos, normal);
                nodesTypes[index] = !validNode ? NodeType.Invalid : NodeType.Free;
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
            NativeArray<RaycastCommand> mainCommands = new NativeArray<RaycastCommand>(expectedGridDimension, Allocator.TempJob);

            JobHandle createNodesHandle = new CalculateRaycastCommandsJob
            {
                commands = mainCommands,
                scanSettings = scanSettings,
            }
            .ScheduleParallel(expectedGridDimension, 64, default(JobHandle));

            // schedule the commands to retrieve the initial hits
            NativeArray<RaycastHit> nodeHits = new NativeArray<RaycastHit>(expectedGridDimension, Allocator.TempJob);
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

        #endregion
    }
}