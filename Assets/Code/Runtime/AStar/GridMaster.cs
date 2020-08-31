using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
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
    public partial class GridMaster : MonoBehaviourSingleton<GridMaster>
    {
        #region Grid Creation Jobs

        /// <summary>
        /// Custom job to parallelize the calculation of launch points and directions for raycasts.
        /// </summary>
        [BurstCompile]
        private struct CalculateRaycastCommandsJob : IJobParallelFor
        {
            [WriteOnly] public NativeArray<RaycastCommand> commands;
            [ReadOnly] private readonly ScanAreaSettings settings;

            public CalculateRaycastCommandsJob(ScanAreaSettings settings)
            {
                commands = new NativeArray<RaycastCommand>(settings.gridWidth * settings.gridDepth, Allocator.TempJob);
                this.settings = settings;
            }

            /// <inheritdoc/>
            public void Execute(int index)
            {
                int row = index / settings.gridWidth;
                int col = index % settings.gridWidth;

                float x = settings.center.x - settings.flooredExtents.x + (col * NodeSize) + NodeHalfSize;
                float y = settings.center.y + settings.extents.y;
                float z = settings.center.z - settings.flooredExtents.z + (row * NodeSize) + NodeHalfSize;

                Vector3 startRayPos = new Vector3(x, y, z);
                commands[index] = new RaycastCommand(startRayPos, Vector3.down, settings.extents.y * 2.0f, settings.mask, 1);
            }

            /// <summary>
            /// Deallocates native datastructures used by the job.
            /// </summary>
            public void Dispose()
            {
                commands.Dispose();
            }
        }

        /// <summary>
        /// The job to create the nodes transforms using the hits position, or if invalid, the
        /// original XZ position of the ray launched.
        /// </summary>
        [BurstCompile]
        private struct CreateNodesJob : IJobParallelFor
        {
            [WriteOnly] private NativeArray<NodeTransform> nodesTransforms;
            [WriteOnly] private NativeArray<NodeType> nodesTypes;
            [ReadOnly] private readonly NativeArray<RaycastHit> hits;
            [ReadOnly] private readonly NativeArray<RaycastCommand> commands;

            public CreateNodesJob(NativeArray<NodeTransform> nodesTransforms, NativeArray<NodeType> nodesTypes, NativeArray<RaycastHit> hits, NativeArray<RaycastCommand> commands)
            {
                this.nodesTransforms = nodesTransforms;
                this.nodesTypes = nodesTypes;
                this.hits = hits;
                this.commands = commands;
            }

            /// <inheritdoc/>
            public void Execute(int index)
            {
                RaycastHit hit = hits[index];
                RaycastCommand command = commands[index];

                // Note: we can't check for collider to be null since reference types are not allowed
                bool validNode = hit.normal != default(Vector3);

                Vector3 pos = validNode ? hit.point : new Vector3(command.from.x, 0.0f, command.from.z);
                Vector3 normal = validNode ? hit.normal : Vector3.up;

                nodesTransforms[index] = new NodeTransform(pos, normal);
                nodesTypes[index] = !validNode ? NodeType.Invalid : NodeType.Free;
            }
        }

        /// <summary>
        /// The job to prepare the bake of the obstacles into the nodes.
        /// </summary>
        [BurstCompile]
        private struct CalculateBoxcastCommandsJob : IJobParallelFor
        {
            [WriteOnly] public NativeArray<BoxcastCommand> commands;
            [ReadOnly] private readonly BakeObstaclesSettings settings;
            [ReadOnly] private NativeArray<NodeTransform> nodesTransforms;

            public CalculateBoxcastCommandsJob(BakeObstaclesSettings settings, NativeArray<NodeTransform> nodesTransforms)
            {
                this.commands = new NativeArray<BoxcastCommand>(nodesTransforms.Length, Allocator.TempJob);
                this.settings = settings;
                this.nodesTransforms = nodesTransforms;
            }

            /// <inheritdoc/>
            public void Execute(int index)
            {
                NodeTransform nt = nodesTransforms[index];

                // start a bit before the node just in case there's an obstacle overlapping a bit
                Vector3 center = nt.Pos - nt.Up * 0.1f;

                // nodes are squares and we don't plan to change it
                float halfWidth = NodeHalfSize * settings.boxNodePercentage;
                float halfDepth = halfWidth;
                Vector3 halfExtents = new Vector3(halfWidth, 0.01f, halfDepth);

                commands[index] = new BoxcastCommand(center, halfExtents, nt.GetRotation(), nt.Up, settings.maxCharacterHeight, settings.mask);
            }

            /// <summary>
            /// Deallocates native datastructures used by the job.
            /// </summary>
            public void Dispose()
            {
                commands.Dispose();
            }
        }

        /// <summary>
        /// The job that takes the results from the boxcasts and makes the node walkable or not
        /// depending on the result.
        /// </summary>
        [BurstCompile]
        private struct BakeObstaclesJob : IJobParallelFor
        {
            private NativeArray<NodeType> nodesTypes;
            [ReadOnly] private NativeArray<RaycastHit> boxcastHits;

            public BakeObstaclesJob(NativeArray<NodeType> nodesTypes, NativeArray<RaycastHit> boxcastHits)
            {
                this.nodesTypes = nodesTypes;
                this.boxcastHits = boxcastHits;
            }

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
        private struct CalculateNeighborsJob : IJobParallelFor
        {
            [WriteOnly, NativeDisableParallelForRestriction] private NativeArray<NodeNeighbor> neighbors;
            [ReadOnly] private readonly NativeArray<NodeTransform> nodesTransforms;
            [ReadOnly] private readonly ScanAreaSettings scanSettings;
            [ReadOnly] private readonly CalculateNeighborSettings neighborSettings;

            public CalculateNeighborsJob(NativeArray<NodeNeighbor> neighbors, NativeArray<NodeTransform> nodesTransforms, ScanAreaSettings scanSettings, CalculateNeighborSettings neighborSettings)
            {
                this.neighbors = neighbors;
                this.nodesTransforms = nodesTransforms;
                this.scanSettings = scanSettings;
                this.neighborSettings = neighborSettings;
            }

            /// <inheritdoc/>
            public void Execute(int index)
            {
                int numNeighbors = NodeNeighbors;

                NativeArray<int> neighborIndices = new NativeArray<int>(numNeighbors, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
                int nodeRow = index / scanSettings.gridWidth;

                int topIndex = index + scanSettings.gridWidth;
                int rightIndex = (index + 1) / scanSettings.gridWidth != nodeRow ? -1 : index + 1;
                int botIndex = index - scanSettings.gridWidth;
                int leftIndex = (index - 1) / scanSettings.gridWidth != nodeRow ? -1 : index - 1;

                // Note: the order is important, its used by some of the PathFinder algorithms
                neighborIndices[0] = topIndex;
                neighborIndices[1] = rightIndex;
                neighborIndices[2] = botIndex;
                neighborIndices[3] = leftIndex;

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

                neighborIndices.Dispose();
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
                float dotRightsAbs = Mathf.Abs(Vector3.Dot(nt.Right, ntn.Right));
                float dotFwdsAbs = Mathf.Abs(Vector3.Dot(nt.Fwd, ntn.Fwd));

                if ((sameCol && dotRightsAbs >= dotThreshold) || (sameRow && dotFwdsAbs >= dotThreshold))
                {
                    // the node can be reached if the distance in height meets the requirements
                    canReachNeighbor = Maths.Dist(nt.Pos.y, ntn.Pos.y) <= neighborSettings.maxWalkableHeightWithStep;
                }

                return canReachNeighbor;
            }
        }

        #endregion

        #region Definitions

        /// <summary>
        /// Structure wrapping the needed settings to scan the area searching for colliders.
        /// </summary>
        private struct ScanAreaSettings
        {
            public readonly int gridWidth;
            public readonly int gridDepth;
            public readonly Vector3 center;
            public readonly Vector3 extents;
            public readonly Vector3 flooredExtents;
            public readonly LayerMask mask;

            public ScanAreaSettings(Vector3 center, Vector3 extents, LayerMask mask)
            {
                this.center = center;
                this.extents = extents;
                this.mask = mask;

                // TODO: Don't allow negative extents when editing the collider
                float restX = extents.x % NodeSize;
                float restY = extents.y % NodeSize;
                float restZ = extents.z % NodeSize;

                float flooredX = extents.x - restX;
                float flooredY = extents.y - restY;
                float flooredZ = extents.z - restZ;

                flooredExtents = new Vector3(flooredX, flooredY, flooredZ);

                this.gridWidth = Mathf.FloorToInt(flooredExtents.x / NodeSize) * 2;
                this.gridDepth = Mathf.FloorToInt(flooredExtents.z / NodeSize) * 2;
            }
        }

        /// <summary>
        /// The settings required by the job that bakes the obstacles using boxcasts.
        /// </summary>
        private struct BakeObstaclesSettings
        {
            public readonly float maxCharacterHeight;
            public readonly float boxNodePercentage;
            public readonly LayerMask mask;

            public BakeObstaclesSettings(float maxCharacterHeight, float boxNodePercentage, LayerMask mask)
            {
                this.maxCharacterHeight = maxCharacterHeight;
                this.boxNodePercentage = boxNodePercentage;
                this.mask = mask;
            }
        }

        /// <summary>
        /// The settings required to calculate the neighbors.
        /// </summary>
        private struct CalculateNeighborSettings
        {
            public readonly float maxWalkableHeightWithSlope;
            public readonly float maxWalkableHeightWithStep;

            public CalculateNeighborSettings(float maxWalkableHeightWithSlope, float maxWalkableHeightWithStep)
            {
                this.maxWalkableHeightWithSlope = maxWalkableHeightWithSlope;
                this.maxWalkableHeightWithStep = maxWalkableHeightWithStep;
            }
        }

        #endregion

        #region Public Attributes

        public const float NodeSize = 1.0f;
        public const float NodeHalfSize = NodeSize * 0.5f;
        public const int NodeNeighbors = (int)NeighborLayout.Four;

        #endregion

        #region Private Attributes

        [Header("Construction settings")]
        [SerializeField] private LayerMask walkableMask = default;

        [SerializeField] private LayerMask obstacleMask = default;

        [SerializeField, Range(0.0f, 5.0f)] private float maxCharacterHeight = 2.0f;
        [SerializeField, Range(0.0f, 1.0f)] private float boxToNodeObstaclePercentage = 0.95f;
        [SerializeField, Range(0.0f, 1.0f)] private float maxWalkableHeightWithSlope = 0.5f;
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
        public NativeArray<NodeTransform> NodesTransforms { get { return nodesTransforms; } }
        public NativeArray<NodeType> NodesTypes { get { return nodesTypes; } }
        public NativeArray<NodeNeighbor> NodesNeighbors { get { return nodesNeighbors; } }
        public Bounds Bounds { get { return scanCollider.bounds; } }

        #endregion

        #region MonoBehaviour Methods

        protected override void Awake()
        {
            base.Awake();
            scanCollider = GetComponent<BoxCollider>();

#if DEBUG_RENDER
            Quaternion rot = Quaternion.LookRotation(Vector3.forward, Vector3.up);
            nodeMesh = RenderUtils.CreateQuadMesh(NodeVisualDebugSize, NodeVisualDebugSize, rot);
            nodeBatcher = new Batcher(nodeMesh, nodeMaterial);
            connectionsMesh = RenderUtils.CreateMeshForProceduralModifications("connectionsMesh", UnityEngine.Rendering.IndexFormat.UInt32);
#endif
            CreateGrid();
        }

#if DEBUG_RENDER

        private void Update()
        {
            nodeBatcher.DoRender();
        }

#endif

        private void OnDestroy()
        {
            if (isGridCreated)
            {
                isGridCreated = false;
                nodesTransforms.Dispose();
                nodesTypes.Dispose();
                nodesNeighbors.Dispose();
            }

#if DEBUG_RENDER
            DisposeDebugNativeDatastructures();
#endif
        }

        #endregion

        #region Initialization Methods

        /// <summary>
        /// Creates the grid of nodes.
        /// </summary>
        public void CreateGrid()
        {
            Bounds scanBounds = scanCollider.bounds;

            // Note: perhaps we might want to snap the extents value when editing the bounding box
            // in the editor?
            ScanAreaSettings scanSettings = new ScanAreaSettings(scanBounds.center, scanBounds.extents, walkableMask);
            gridWidth = scanSettings.gridWidth;
            gridDepth = scanSettings.gridDepth;

            int gridDimension = gridWidth * gridDepth;

            if (isGridCreated)
            {
                nodesTransforms.Dispose();
                nodesTypes.Dispose();
                nodesNeighbors.Dispose();
            }

            nodesTransforms = new NativeArray<NodeTransform>(gridDimension, Allocator.Persistent);
            nodesTypes = new NativeArray<NodeType>(gridDimension, Allocator.Persistent);
            nodesNeighbors = new NativeArray<NodeNeighbor>(gridDimension * NodeNeighbors, Allocator.Persistent);

            // calculate the raycasts commands
            CalculateRaycastCommandsJob calculateCommandsJob = new CalculateRaycastCommandsJob(scanSettings);
            JobHandle calculateCommandsHandle = calculateCommandsJob.Schedule(gridDimension, 32);

            // schedule the commands to retrieve the hits
            NativeArray<RaycastHit> nodeHits = new NativeArray<RaycastHit>(gridDimension, Allocator.TempJob);
            JobHandle raycastCommandHandle = RaycastCommand.ScheduleBatch(calculateCommandsJob.commands, nodeHits, 1, calculateCommandsHandle);

            // build the nodes using the received hits
            CreateNodesJob createNodesJob = new CreateNodesJob(nodesTransforms, nodesTypes, nodeHits, calculateCommandsJob.commands);
            JobHandle createNodesHandle = createNodesJob.Schedule(gridDimension, 32, raycastCommandHandle);

            // calculate the boxcast to bake obstacles
            BakeObstaclesSettings bakeObstaclesSettings = new BakeObstaclesSettings(maxCharacterHeight, boxToNodeObstaclePercentage, obstacleMask);
            CalculateBoxcastCommandsJob calculateBoxcastCommandsJob = new CalculateBoxcastCommandsJob(bakeObstaclesSettings, nodesTransforms);
            JobHandle calculateBoxcastHandle = calculateBoxcastCommandsJob.Schedule(gridDimension, 32, createNodesHandle);

            NativeArray<RaycastHit> obstacleHits = new NativeArray<RaycastHit>(gridDimension, Allocator.TempJob);
            JobHandle boxcastCommandHandle = BoxcastCommand.ScheduleBatch(calculateBoxcastCommandsJob.commands, obstacleHits, 1, calculateBoxcastHandle);

            // prepare the bake obstacles job
            BakeObstaclesJob bakeObstaclesJob = new BakeObstaclesJob(nodesTypes, obstacleHits);
            JobHandle bakeObstaclesHandle = bakeObstaclesJob.Schedule(gridDimension, 32, boxcastCommandHandle);

            // now calculate the neighbors
            CalculateNeighborSettings neighborSettings = new CalculateNeighborSettings(maxWalkableHeightWithSlope, maxWalkableHeightWithStep);
            CalculateNeighborsJob calculateNeighborsJob = new CalculateNeighborsJob(nodesNeighbors, nodesTransforms, scanSettings, neighborSettings);
            JobHandle calculateNeighborsHandle = calculateNeighborsJob.Schedule(gridDimension, 32, bakeObstaclesHandle);

            // wait to complete all the scheduled stuff
            calculateNeighborsHandle.Complete();

            calculateCommandsJob.Dispose();
            calculateBoxcastCommandsJob.Dispose();
            nodeHits.Dispose();
            obstacleHits.Dispose();

#if DEBUG_RENDER
            RecalculateDebug();
#endif
            isGridCreated = true;
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
            float restY = extents.y % NodeSize;
            float restZ = extents.z % NodeSize;

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

            int row = Mathf.FloorToInt(localPos.z / NodeSize);
            int col = Mathf.FloorToInt(localPos.x / NodeSize);

            return row * gridWidth + col;
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