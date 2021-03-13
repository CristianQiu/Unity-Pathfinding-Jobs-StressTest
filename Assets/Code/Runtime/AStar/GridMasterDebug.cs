using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Rendering;
using UnityLibrary;

namespace AStar
{
    /// <summary>
    /// Class in charge of rendering the grid with the data from the GridMaster.
    /// </summary>
    public class GridMasterDebug : MonoBehaviourSingleton<GridMasterDebug>
    {
#if DEBUG_RENDER

        #region Jobs

        /// <summary>
        /// Job to calculate the required data for the jobified quad renderer.
        /// </summary>
        [BurstCompile]
        private struct CalculateNodeQuadsDataJob : IJobFor
        {
            [WriteOnly] public NativeArray<float3> quadPositions;
            [WriteOnly] public NativeArray<float3> quadNormals;
            [WriteOnly] public NativeArray<float2> quadDimensions;
            [WriteOnly] public NativeArray<quaternion> quadRotations;
            [WriteOnly] public NativeArray<Color32> quadColors;

            [ReadOnly] public NativeArray<NodeTransform> nodeTransforms;
            [ReadOnly] public NativeArray<NodeType> nodesTypes;

            public Color32 invalidColor;
            public Color32 walkableColor;
            public Color32 nonWalkableColor;
            public float2 dimension;

            /// <inheritdoc/>
            public void Execute(int index)
            {
                quadPositions[index] = nodeTransforms[index].pos + (nodeTransforms[index].up * NodeVisualNormalOffset);
                quadNormals[index] = nodeTransforms[index].up;
                quadDimensions[index] = dimension;
                quadRotations[index] = nodeTransforms[index].GetRotation();

                switch (nodesTypes[index])
                {
                    case NodeType.Free:
                        quadColors[index] = walkableColor;
                        break;

                    case NodeType.OccupiedByCharacter:
                        quadColors[index] = walkableColor;
                        break;

                    case NodeType.OccupiedByObstacle:
                        quadColors[index] = nonWalkableColor;
                        break;

                    case NodeType.Invalid:
                        quadColors[index] = invalidColor;
                        break;

                    default:
                        quadColors[index] = invalidColor;
                        break;
                }
            }
        }

        /// <summary>
        /// The job that calculates the connection vertices for the nodes, so that they can be
        /// displayed with a mesh.
        /// </summary>
        [BurstCompile]
        private struct CalculateConnectionMeshJob : IJobFor
        {
            [WriteOnly, NativeDisableParallelForRestriction] public NativeArray<float3> vertices;
            [WriteOnly, NativeDisableParallelForRestriction] public NativeArray<int> indices;

            [ReadOnly] public NativeArray<NodeTransform> nodesTransforms;
            [ReadOnly] public NativeArray<NodeNeighbor> nodesNeighbors;

            public int numNeighbors;

            /// <inheritdoc/>
            public void Execute(int index)
            {
                int startNeighborIndex = index * numNeighbors;
                int endNeighborIndex = startNeighborIndex + numNeighbors;

                NodeTransform nt = nodesTransforms[index];

                for (int i = startNeighborIndex; i < endNeighborIndex; i++)
                {
                    int neighborIndex = nodesNeighbors[i].neighborIndex;
                    bool valid = nodesNeighbors[i].isValid;

                    int vertexIndex = index * numNeighbors * 2 + ((i - startNeighborIndex) * 2);

                    indices[vertexIndex] = vertexIndex;
                    indices[vertexIndex + 1] = vertexIndex + 1;

                    if (!valid)
                    {
                        vertices[vertexIndex] = float3.zero;
                        vertices[vertexIndex + 1] = float3.zero;
                    }
                    else
                    {
                        NodeTransform ntn = nodesTransforms[neighborIndex];

                        vertices[vertexIndex] = nt.pos + (nt.up * NodeConnectionVisualNormalOffset);
                        vertices[vertexIndex + 1] = math.lerp(nt.pos + (nt.up * NodeConnectionVisualNormalOffset), ntn.pos + (ntn.up * NodeConnectionVisualNormalOffset), 0.5f);
                    }
                }
            }
        }

        #endregion

        #region Private Attributes

        private const float NodeVisualDebugSize = GridMaster.NodeSize * 0.875f;
        private const float NodeVisualNormalOffset = 0.001f;
        private const float NodeConnectionVisualNormalOffset = 0.002f;

        [Header("Nodes")]
        [SerializeField] private JobifiedQuadRenderer nodesRenderer = null;
        [SerializeField] private Color32 invalidNodeColor = new Color32(0, 0, 0, 208);
        [SerializeField] private Color32 walkableNodeColor = new Color32(128, 128, 128, 208);
        [SerializeField] private Color32 nonWalkableNodeColor = new Color32(255, 0, 0, 208);
        private bool showNodes;

        [Header("Connections")]
        [SerializeField] private Material connectionsMaterial = null;
        private bool showConnections;
        private Mesh connectionsMesh;
        private MeshRenderer connectionsMeshRenderer;
        private NativeArray<float3> connectionsMeshVertices;
        private NativeArray<int> connectionsMeshIndices;

        private GridMaster gm;

        #endregion

        #region Properties

        protected override bool DestroyOnLoad { get { return true; } }

        public bool ShowNodes
        {
            get { return showNodes; }
            set
            {
                showNodes = value;
                UpdateShowDebug();
            }
        }

        public bool ShowConnections
        {
            get { return showConnections; }
            set
            {
                showConnections = value;
                UpdateShowDebug();
            }
        }

        #endregion

        #region MonoBehaviour Methods

        private void Start()
        {
            gm = GridMaster.Instance;
            gm.OnGridCreation += RecalculateDebug;

            connectionsMesh = RenderUtils.CreateMeshForProceduralModifications("connectionsMesh", IndexFormat.UInt32);

            MeshFilter filter = Utils.GetOrAddComponent<MeshFilter>(gm.transform, out bool createdFilter);
            filter.sharedMesh = connectionsMesh;

            connectionsMeshRenderer = Utils.GetOrAddComponent<MeshRenderer>(gm.transform, out bool createdRenderer);
            connectionsMeshRenderer.sharedMaterial = connectionsMaterial;
            connectionsMeshRenderer.shadowCastingMode = ShadowCastingMode.Off;
            connectionsMeshRenderer.lightProbeUsage = LightProbeUsage.Off;
            connectionsMeshRenderer.reflectionProbeUsage = ReflectionProbeUsage.Off;
        }

        private void OnDestroy()
        {
            gm.OnGridCreation -= RecalculateDebug;

            if (connectionsMesh != null)
                Destroy(connectionsMesh);

            DeallocateNativeDatastructures();
        }

        #endregion

        #region Methods

        /// <summary>
        /// Updates showing the debug according to the internal state of the object.
        /// </summary>
        private void UpdateShowDebug()
        {
            connectionsMeshRenderer.enabled = showConnections;
            nodesRenderer.Show(showNodes);
        }

        /// <summary>
        /// Recalculates the grid debugging, both the connections and the nodes.
        /// </summary>
        private void RecalculateDebug()
        {
            DeallocateNativeDatastructures();

            int numNodes = gm.GridDepth * gm.GridWidth;
            int arrayLength = numNodes * GridMaster.NodeNumNeighbors * 2;

            connectionsMeshVertices = new NativeArray<float3>(arrayLength, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
            connectionsMeshIndices = new NativeArray<int>(arrayLength, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);

            JobifiedQuadRenderer.QuadData quadNodesData = new JobifiedQuadRenderer.QuadData(numNodes);

            JobHandle nodeQuadsDeps = new CalculateNodeQuadsDataJob
            {
                quadPositions = quadNodesData.quadPositions,
                quadNormals = quadNodesData.quadNormals,
                quadDimensions = quadNodesData.quadDimensions,
                quadRotations = quadNodesData.quadRotations,
                quadColors = quadNodesData.quadColors,
                nodeTransforms = gm.NodesTransforms,
                nodesTypes = gm.NodesTypes,
                invalidColor = invalidNodeColor,
                walkableColor = walkableNodeColor,
                nonWalkableColor = nonWalkableNodeColor,
                dimension = new float2(NodeVisualDebugSize, NodeVisualDebugSize),
            }
            .ScheduleParallel(numNodes, 64, default(JobHandle));

            nodeQuadsDeps = nodesRenderer.ScheduleBuildMesh(quadNodesData, 64, nodeQuadsDeps);
            JobHandle disposeNodeDeps = quadNodesData.Dispose(nodeQuadsDeps);

            CalculateConnectionMeshJob calcConnectionsMeshJob = new CalculateConnectionMeshJob
            {
                vertices = connectionsMeshVertices,
                indices = connectionsMeshIndices,
                nodesTransforms = gm.NodesTransforms,
                nodesNeighbors = gm.NodesNeighbors,
                numNeighbors = GridMaster.NodeNumNeighbors,
            };

            JobHandle calcConnectionMeshHandle = calcConnectionsMeshJob.ScheduleParallel(numNodes, 64, default(JobHandle));
            JobHandle.CompleteAll(ref calcConnectionMeshHandle, ref nodeQuadsDeps);

            connectionsMesh.SetVertices(calcConnectionsMeshJob.vertices);
            connectionsMesh.SetIndices(calcConnectionsMeshJob.indices, MeshTopology.Lines, 0);
            nodesRenderer.UpdateMesh();

            UpdateShowDebug();

            disposeNodeDeps.Complete();
        }

        /// <summary>
        /// Dispose native datastructures used to debug.
        /// </summary>
        private void DeallocateNativeDatastructures()
        {
            if (connectionsMeshVertices.IsCreated)
                connectionsMeshVertices.Dispose();

            if (connectionsMeshIndices.IsCreated)
                connectionsMeshIndices.Dispose();
        }

        #endregion

#endif
    }
}