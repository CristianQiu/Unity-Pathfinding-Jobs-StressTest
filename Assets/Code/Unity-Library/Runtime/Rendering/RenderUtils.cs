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
            mesh.indexFormat = bufferFormat;
            mesh.MarkDynamic();

            return mesh;
        }

        #endregion
    }
}