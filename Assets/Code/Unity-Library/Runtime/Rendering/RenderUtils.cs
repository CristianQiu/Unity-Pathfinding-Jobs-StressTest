using UnityEngine;
using UnityEngine.Rendering;

namespace UnityLibrary
{
    /// <summary>
    /// Simple class for rendering utilities. Meshes, shaders, etc...
    /// </summary>
    public static class RenderUtils
    {
        #region Mesh Methods

        /// <summary>
        /// Create a mesh with the given name and buffer format. The mesh is marked dynamic for
        /// procedural modifications.
        /// </summary>
        /// <param name="meshName"></param>
        /// <param name="bufferFormat"></param>
        /// <returns></returns>
        public static Mesh CreateMeshForProceduralModifications(string meshName, IndexFormat bufferFormat)
        {
            Mesh mesh = new Mesh();
            mesh.name = meshName;
            mesh.MarkDynamic();
            mesh.indexFormat = bufferFormat;

            return mesh;
        }

        /// <summary>
        /// Gets a new quad mesh with the given rotation.
        /// </summary>
        /// <param name="width"></param>
        /// <param name="height"></param>
        /// <param name="rotation"></param>
        /// <returns></returns>
        public static Mesh CreateQuadMesh(float width, float height, Quaternion rotation)
        {
            float halfWidth = width * 0.5f;
            float halfHeight = height * 0.5f;

            Mesh quad = new Mesh();

            Vector3[] vertices = new Vector3[4];
            vertices[0] = rotation * new Vector3(-halfWidth, 0.0f, -halfHeight);
            vertices[1] = rotation * new Vector3(-halfWidth, 0.0f, halfHeight);
            vertices[2] = rotation * new Vector3(halfWidth, 0.0f, halfHeight);
            vertices[3] = rotation * new Vector3(halfWidth, 0.0f, -halfHeight);

            quad.vertices = vertices;

            int[] triangles = new int[6];
            triangles[0] = 0;
            triangles[1] = 1;
            triangles[2] = 3;
            triangles[3] = 1;
            triangles[4] = 2;
            triangles[5] = 3;

            quad.triangles = triangles;

            Vector2[] uv = new Vector2[4];
            uv[0] = new Vector2(0, 0);
            uv[1] = new Vector2(0, 1);
            uv[2] = new Vector2(1, 1);
            uv[3] = new Vector2(1, 0);

            quad.uv = uv;

            quad.RecalculateNormals();
            quad.RecalculateBounds();

            return quad;
        }

        #endregion
    }
}