using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Profiling;
using UnityEngine.Rendering;

namespace UnityLibrary
{
    /// <summary>
    /// Base class that is thought to be used by any class that can make custom procedural meshes
    /// using the jobs system. Its mainly for setup stuff since there is a great deal of code that
    /// is always the same. As a very important note, this class uses a fixed buffer for the
    /// vertices and indices and other arrays used by the mesh, which is updated by notifying the
    /// last index that should be considered.
    /// </summary>
    [DisallowMultipleComponent]
    [RequireComponent(typeof(MeshFilter), typeof(MeshRenderer))]
    public abstract class BaseJobifiedProceduralMeshRenderer : MonoBehaviour
    {
        #region Private Attributes

        protected NativeArray<float3> vertices;
        protected NativeArray<int> tris;
        protected NativeArray<float3> normals;
        protected NativeArray<float2> uvs;
        protected NativeArray<Color32> colors;

        protected MeshRenderer meshRenderer;
        private MeshFilter meshFilter;
        private Mesh mesh;

        private CustomSampler sampler = CustomSampler.Create("BaseJobifiedProceduralMeshRenderer.UpdateMesh()");

        #endregion

        #region Properties

        protected abstract string MeshName { get; }
        protected abstract IndexFormat MeshIndexformat { get; }
        protected abstract bool UseNormals { get; }
        protected abstract bool UseUvs { get; }
        protected abstract bool UseVertexColors { get; }
        protected virtual int NumPreallocVertices { get { return MeshIndexformat == IndexFormat.UInt16 ? 8192 : 262144; } }
        protected virtual int NumPreallocIndices { get { return NumPreallocVertices * 3; } }
        protected virtual ShadowCastingMode ShadowMode { get { return ShadowCastingMode.Off; } }
        protected virtual ReflectionProbeUsage ReflectionProbeMode { get { return ReflectionProbeUsage.Off; } }
        protected virtual LightProbeUsage LightProbeMode { get { return LightProbeUsage.Off; } }

        #endregion

        #region MonoBehaviour Methods

        protected virtual void Awake()
        {
            meshRenderer = GetComponent<MeshRenderer>();
            meshFilter = GetComponent<MeshFilter>();
        }

        protected virtual void Start()
        {
            mesh = RenderUtils.CreateMeshForProceduralModifications(MeshName, MeshIndexformat);
            meshFilter.sharedMesh = mesh;
            AllocateNativeArrays();
        }

        protected virtual void OnDestroy()
        {
            if (mesh != null)
                Destroy(mesh);

            DisposeNativeArrays();
        }

        private void Reset()
        {
            SetupMeshRendererSettings();
        }

        #endregion

        #region Initialization Methods

        /// <summary>
        /// Allocates the native arrays.
        /// </summary>
        protected virtual void AllocateNativeArrays()
        {
            vertices = new NativeArray<float3>(NumPreallocVertices, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
            tris = new NativeArray<int>(NumPreallocIndices, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
            if (UseNormals)
                normals = new NativeArray<float3>(NumPreallocVertices, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
            if (UseUvs)
                uvs = new NativeArray<float2>(NumPreallocVertices, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
            if (UseVertexColors)
                colors = new NativeArray<Color32>(NumPreallocVertices, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
        }

        /// <summary>
        /// Deallocates the native data structures used by the mesh.
        /// </summary>
        protected virtual void DisposeNativeArrays()
        {
            if (vertices.IsCreated)
                vertices.Dispose();
            if (tris.IsCreated)
                tris.Dispose();
            if (normals.IsCreated)
                normals.Dispose();
            if (uvs.IsCreated)
                uvs.Dispose();
            if (colors.IsCreated)
                colors.Dispose();
        }

        /// <summary>
        /// Setups the mesh renderer configuration according to the attributes chosen by the derived class.
        /// </summary>
        protected void SetupMeshRendererSettings()
        {
            MeshRenderer mr = GetComponent<MeshRenderer>();
            mr.shadowCastingMode = ShadowMode;
            mr.reflectionProbeUsage = ReflectionProbeMode;
            mr.lightProbeUsage = LightProbeMode;
        }

        #endregion

        #region Methods

        /// <summary>
        /// Shows or hides the mesh.
        /// </summary>
        /// <param name="show"></param>
        public void Show(bool show)
        {
            meshRenderer.enabled = show;
        }

        /// <summary>
        /// Updates the mesh by setting its data given the last valid vertex index and the last
        /// valid triangle index. Be aware that by default the data is set starting from index 0.
        /// </summary>
        /// <param name="lastValidVertexIndex"></param>
        /// <param name="lastValidTriIndex"></param>
        protected void UpdateMesh(int lastValidVertexIndex, int lastValidTriIndex)
        {
            sampler.Begin();

            // Note: this might be called in between job completions so it could be moved away.
            mesh.Clear(false);

            MeshUpdateFlags updateFlags = ~MeshUpdateFlags.Default;

            mesh.SetVertices(vertices, 0, lastValidVertexIndex, updateFlags);
            mesh.SetIndices(tris, 0, lastValidTriIndex, MeshTopology.Triangles, 0, false);
            if (UseNormals)
                mesh.SetNormals(normals, 0, lastValidVertexIndex, updateFlags);
            if (UseUvs)
                mesh.SetUVs(0, uvs, 0, lastValidVertexIndex, updateFlags);
            if (UseVertexColors)
                mesh.SetColors(colors, 0, lastValidVertexIndex, updateFlags);

            mesh.RecalculateBounds();

            sampler.End();
        }

        #endregion
    }
}