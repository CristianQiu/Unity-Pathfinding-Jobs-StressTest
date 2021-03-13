using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Rendering;

namespace UnityLibrary
{
    /// <summary>
    /// Class that is able to make a procedural mesh from a given set of quads. It is jobified so
    /// scaling to a very high number of quads is completely reasonable. It currently relies on a
    /// material that has a shader with vertex color support, so make sure that if you intend to use
    /// vertex coloring change the default material.
    /// </summary>
    [DisallowMultipleComponent]
    [RequireComponent(typeof(MeshFilter), typeof(MeshRenderer))]
    public class JobifiedQuadRenderer : BaseJobifiedProceduralMeshRenderer
    {
        #region Defs

        /// <summary>
        /// Struct to wrap the quad data and avoid boilerplate.
        /// </summary>
        public struct QuadData
        {
            public readonly NativeArray<float3> quadPositions;
            public readonly NativeArray<float3> quadNormals;
            public readonly NativeArray<float2> quadDimensions;
            public readonly NativeArray<quaternion> quadRotations;
            public readonly NativeArray<Color32> quadColors;

            public QuadData(int numQuads)
            {
                quadPositions = new NativeArray<float3>(numQuads, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);
                quadNormals = new NativeArray<float3>(numQuads, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);
                quadDimensions = new NativeArray<float2>(numQuads, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);
                quadRotations = new NativeArray<quaternion>(numQuads, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);
                quadColors = new NativeArray<Color32>(numQuads, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);
            }

            /// <summary>
            /// Schedules disposing the native data structures when the dependency is completed.
            /// </summary>
            /// <param name="inputDeps"></param>
            /// <returns></returns>
            public JobHandle Dispose(JobHandle inputDeps)
            {
                JobHandle disposeDeps = JobHandle.CombineDependencies(quadPositions.Dispose(inputDeps), quadNormals.Dispose(inputDeps), quadDimensions.Dispose(inputDeps));

                return JobHandle.CombineDependencies(disposeDeps, quadRotations.Dispose(inputDeps), quadColors.Dispose(inputDeps));
            }

            /// <summary>
            /// Disposes the native data structures.
            /// </summary>
            public void Dispose()
            {
                quadPositions.Dispose();
                quadNormals.Dispose();
                quadDimensions.Dispose();
                quadRotations.Dispose();
                quadColors.Dispose();
            }
        }

        #endregion

        #region Jobs

        /// <summary>
        /// The job that fills the mesh needed data structures from the given set of quads.
        /// </summary>
        [BurstCompile]
        private struct BuildMeshFromQuadsJob : IJobFor
        {
            [WriteOnly, NativeDisableParallelForRestriction] public NativeArray<float3> vertices;
            [WriteOnly, NativeDisableParallelForRestriction] public NativeArray<int> tris;
            [WriteOnly, NativeDisableParallelForRestriction] public NativeArray<float3> normals;
            [WriteOnly, NativeDisableParallelForRestriction] public NativeArray<Color32> colors;

            [ReadOnly] public NativeArray<float3> quadPositions;
            [ReadOnly] public NativeArray<float3> quadNormals;
            [ReadOnly] public NativeArray<float2> quadDimensions;
            [ReadOnly] public NativeArray<quaternion> quadRotations;
            [ReadOnly] public NativeArray<Color32> quadColors;

            /// <inheritdoc/>
            public void Execute(int index)
            {
                int startVertex = index * 4;

                float3 fwd = new float3(0.0f, 0.0f, 0.5f * quadDimensions[index].y);
                float3 right = new float3(0.5f * quadDimensions[index].x, 0.0f, 0.0f);

                vertices[startVertex] = math.mul(quadRotations[index], fwd + right) + quadPositions[index];
                vertices[startVertex + 1] = math.mul(quadRotations[index], -fwd + right) + quadPositions[index];
                vertices[startVertex + 2] = math.mul(quadRotations[index], -fwd - right) + quadPositions[index];
                vertices[startVertex + 3] = math.mul(quadRotations[index], fwd - right) + quadPositions[index];

                int startTriIndex = index * 6;

                tris[startTriIndex] = startVertex;
                tris[startTriIndex + 1] = startVertex + 1;
                tris[startTriIndex + 2] = startVertex + 2;

                tris[startTriIndex + 3] = startVertex + 2;
                tris[startTriIndex + 4] = startVertex + 3;
                tris[startTriIndex + 5] = startVertex;

                normals[startVertex] = quadNormals[index];
                normals[startVertex + 1] = quadNormals[index];
                normals[startVertex + 2] = quadNormals[index];
                normals[startVertex + 3] = quadNormals[index];

                colors[startVertex] = quadColors[index];
                colors[startVertex + 1] = quadColors[index];
                colors[startVertex + 2] = quadColors[index];
                colors[startVertex + 3] = quadColors[index];
            }
        }

        #endregion

        #region Private Attributes

        private int lastScheduledValidVertex;
        private int lastScheduledValidTriIndex;

        #endregion

        #region Properties

        protected override string MeshName { get { return "JobifiedQuadMesh"; } }
        protected override IndexFormat MeshIndexformat { get { return IndexFormat.UInt32; } }
        protected override bool UseNormals { get { return true; } }
        protected override bool UseUvs { get { return false; } }
        protected override bool UseVertexColors { get { return true; } }

        #endregion

        #region Methods

        /// <summary>
        /// Schedules the job to refresh the mesh data structures data with the given quads data.
        /// </summary>
        /// <param name="quadData"></param>
        /// <param name="innerBatchLoopCount"></param>
        /// <param name="dependsOn"></param>
        /// <returns></returns>
        public JobHandle ScheduleBuildMesh(QuadData quadData, int innerBatchLoopCount, JobHandle dependsOn = default)
        {
            return ScheduleBuildMesh(quadData.quadPositions, quadData.quadNormals, quadData.quadDimensions, quadData.quadRotations, quadData.quadColors, innerBatchLoopCount, dependsOn);
        }

        /// <summary>
        /// Schedules the job to refresh the mesh data structures data with the given quads data.
        /// </summary>
        /// <param name="quadPositions"></param>
        /// <param name="quadNormals"></param>
        /// <param name="quadDimensions"></param>
        /// <param name="quadRotations"></param>
        /// <param name="quadColors"></param>
        /// <param name="innerBatchLoopCount"></param>
        /// <param name="dependsOn"></param>
        /// <returns></returns>
        public JobHandle ScheduleBuildMesh(NativeArray<Vector3> quadPositions, NativeArray<Vector3> quadNormals, NativeArray<Vector2> quadDimensions, NativeArray<Quaternion> quadRotations, NativeArray<Color32> quadColors, int innerBatchLoopCount, JobHandle dependsOn = default)
        {
            NativeArray<float3> qPoss = quadPositions.Reinterpret<float3>();
            NativeArray<float3> qNorms = quadNormals.Reinterpret<float3>();
            NativeArray<float2> qDims = quadDimensions.Reinterpret<float2>();
            NativeArray<quaternion> qRots = quadRotations.Reinterpret<quaternion>();

            return ScheduleBuildMesh(qPoss, qNorms, qDims, qRots, quadColors, innerBatchLoopCount, dependsOn);
        }

        /// <summary>
        /// Schedules the job to refresh the mesh data structures data with the given quads data.
        /// </summary>
        /// <param name="quadPositions"></param>
        /// <param name="quadNormals"></param>
        /// <param name="quadDimensions"></param>
        /// <param name="quadRotations"></param>
        /// <param name="quadColors"></param>
        /// <param name="innerBatchLoopCount"></param>
        /// <param name="dependsOn"></param>
        /// <returns></returns>
        public JobHandle ScheduleBuildMesh(NativeArray<float3> quadPositions, NativeArray<float3> quadNormals, NativeArray<float2> quadDimensions, NativeArray<quaternion> quadRotations, NativeArray<Color32> quadColors, int innerBatchLoopCount, JobHandle dependsOn = default)
        {
            bool ok = quadPositions.Length == quadNormals.Length && quadPositions.Length == quadDimensions.Length && quadPositions.Length == quadRotations.Length && quadPositions.Length == quadColors.Length;
            Asserter.Assert(ok, "The number of quad positions, normals, dimensions, rotations and colors must be the same!");

            lastScheduledValidVertex = quadPositions.Length * 4;
            lastScheduledValidTriIndex = quadPositions.Length * 6;

            return new BuildMeshFromQuadsJob
            {
                vertices = vertices,
                tris = tris,
                normals = normals,
                colors = colors,
                quadPositions = quadPositions,
                quadNormals = quadNormals,
                quadDimensions = quadDimensions,
                quadRotations = quadRotations,
                quadColors = quadColors,
            }
            .ScheduleParallel(quadPositions.Length, innerBatchLoopCount, dependsOn);
        }

        /// <summary>
        /// Updates the mesh with the current state of the mesh data structures. This should be
        /// called after calling complete on the job handle that ScheduleBuildMesh returns.
        /// </summary>
        public void UpdateMesh()
        {
            UpdateMesh(lastScheduledValidVertex, lastScheduledValidTriIndex);
        }

        #endregion
    }
}