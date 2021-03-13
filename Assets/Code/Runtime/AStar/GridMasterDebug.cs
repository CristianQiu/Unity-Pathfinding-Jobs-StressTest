using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using UnityEngine;
using UnityEngine.Rendering;
using UnityLibrary;

namespace AStar
{
    /// <summary>
    /// This is the partial class supporting GridMaster.cs. It holds all the definitions used by
    /// debugging to avoid compiling them. The logic however, is still called on the other part of
    /// the class.
    /// </summary>
    public partial class GridMaster
    {
#if DEBUG_RENDER

        #region Jobs

        /// <summary>
        /// The job that calculates the connection vertices for the nodes, so that they can be
        /// displayed with a mesh.
        /// </summary>
        [BurstCompile]
        private struct CalculateConnectionMeshJob : IJobParallelFor
        {
            [ReadOnly] private readonly int numNeighbors;
            [ReadOnly] private readonly NativeArray<NodeTransform> nodesTransforms;
            [ReadOnly] private readonly NativeArray<NodeNeighbor> nodesNeighbors;
            [WriteOnly, NativeDisableParallelForRestriction] public NativeArray<Vector3> vertices;
            [WriteOnly, NativeDisableParallelForRestriction] public NativeArray<int> indices;

            public CalculateConnectionMeshJob(int numNeighbors, NativeArray<NodeTransform> nodesTransforms, NativeArray<NodeNeighbor> nodesNeighbors, NativeArray<Vector3> vertices, NativeArray<int> indices)
            {
                this.numNeighbors = numNeighbors;
                this.nodesTransforms = nodesTransforms;
                this.nodesNeighbors = nodesNeighbors;
                this.vertices = vertices;
                this.indices = indices;
            }

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
                        vertices[vertexIndex] = Vector3.zero;
                        vertices[vertexIndex + 1] = Vector3.zero;
                    }
                    else
                    {
                        NodeTransform ntn = nodesTransforms[neighborIndex];

                        vertices[vertexIndex] = nt.Pos + (nt.Up * NodeVisualNormalOffset);
                        vertices[vertexIndex + 1] = Vector3.Lerp(nt.Pos + (nt.Up * NodeVisualNormalOffset), ntn.Pos + (ntn.Up * NodeVisualNormalOffset), 0.5f);
                    }
                }
            }
        }

        #endregion

        #region Private Attributes

        private const float NodeVisualDebugSize = NodeSize * 0.8f;
        private const float NodeVisualNormalOffset = 0.001f;

        [Header("Debug settings")]
        [SerializeField] private bool showNodes = true;
        [SerializeField] private Material nodeMaterial = null;
        [SerializeField] private Color32 invalidNodeColor = new Color32(0, 0, 0, 192);
        [SerializeField] private Color32 walkableNodeColor = new Color32(128, 128, 128, 128);
        [SerializeField] private Color32 nonWalkableNodeColor = new Color32(255, 0, 0, 192);

        [SerializeField] private bool showNodesConnections = true;
        [SerializeField] private Material nodeConnectionsMaterial = null;

        private Mesh nodeMesh;
        private Batcher nodeBatcher;

        private Mesh connectionsMesh;
        private NativeArray<Vector3> connectionsMeshVertices;
        private NativeArray<int> connectionsMeshIndices;

        #endregion

        #region Properties

        public bool ShowNodes
        {
            get { return showNodes; }
            set
            {
                showNodes = value;
                RecalculateDebug();
            }
        }

        public bool ShowNodesConnections
        {
            get { return showNodesConnections; }
            set
            {
                showNodesConnections = value;
                RecalculateDebug();
            }
        }

        #endregion

        #region Methods

        /// <summary>
        /// Adds the nodes debugging.
        /// </summary>
        public void RecalculateDebug()
        {
            DisposeDebugNativeDatastructures();
            int numNodes = gridDepth * gridWidth;

            // prepare the job that calculates the vertices for the neighbor connection lines
            int arrayLength = numNodes * NodeNeighbors * 2;
            connectionsMeshVertices = new NativeArray<Vector3>(arrayLength, Allocator.Persistent);
            connectionsMeshIndices = new NativeArray<int>(arrayLength, Allocator.Persistent);

            CalculateConnectionMeshJob calcConnectionsMeshJob = new CalculateConnectionMeshJob(NodeNeighbors, nodesTransforms, nodesNeighbors, connectionsMeshVertices, connectionsMeshIndices);
            JobHandle calcConnectionMeshHandle = calcConnectionsMeshJob.Schedule(numNodes, 8);

            // do other required stuff before calling complete so we have actual parallelism
            MeshRenderer mr = Utils.GetOrAddComponent<MeshRenderer>(transform, out bool createdRenderer);
            mr.shadowCastingMode = ShadowCastingMode.Off;
            mr.sharedMaterial = nodeConnectionsMaterial;
            mr.lightProbeUsage = LightProbeUsage.Off;
            mr.reflectionProbeUsage = ReflectionProbeUsage.Off;
            mr.enabled = showNodesConnections;

            MeshFilter filter = Utils.GetOrAddComponent<MeshFilter>(transform, out bool createdFilter);
            filter.sharedMesh = connectionsMesh;

            // the nodes themselves
            nodeBatcher.Clear();

            if (showNodes)
            {
                for (int i = 0; i < numNodes; i++)
                {
                    NodeTransform nt = nodesTransforms[i];
                    NodeType nodeType = nodesTypes[i];

                    Color32 c;

                    if (nodeType == NodeType.Invalid)
                        c = invalidNodeColor;
                    else if (nodeType == NodeType.OccupiedByObstacle)
                        c = nonWalkableNodeColor;
                    else
                        c = walkableNodeColor;

                    Vector3 pos = nt.Pos + (nt.Up * NodeVisualNormalOffset);
                    Matrix4x4 trs = Matrix4x4.TRS(pos, nt.GetRotation(), Vector3.one);

                    // batch each node quad debug
                    nodeBatcher.AddItem(c, trs);
                }
            }

            calcConnectionMeshHandle.Complete();

            // set the mesh using the results of the job
            connectionsMesh.SetVertices(calcConnectionsMeshJob.vertices);
            connectionsMesh.SetIndices(calcConnectionsMeshJob.indices, MeshTopology.Lines, 0);
        }

        /// <summary>
        /// Dispose native datastructures used by debugging.
        /// </summary>
        private void DisposeDebugNativeDatastructures()
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