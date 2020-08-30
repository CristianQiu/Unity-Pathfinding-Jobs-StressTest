using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering;

namespace UnityLibrary
{
    /// <summary>
    /// Class that can render batches with Graphics.DrawMeshInstanced with different colors, up to a
    /// maximum amount of different colors defined. One draw call is used per 1023 instances per color.
    /// </summary>
    public class Batcher
    {
        #region Definitions

        /// <summary>
        /// Class to be used to render the mesh with the given color and as many times as number of matrices.
        /// </summary>
        private class BatchInfo
        {
            public Color32 color;

            private int currListIndex;
            private int currArrayIndex;

            private readonly List<Matrix4x4[]> batches;

            public List<Matrix4x4[]> Batches { get { return batches; } }

            public BatchInfo(Color32 color)
            {
                this.color = color;
                batches = new List<Matrix4x4[]>();
            }

            /// <summary>
            /// Adds an object to be rendered with the given matrix.
            /// </summary>
            /// <param name="matrix"></param>
            public void AddMatrix(Matrix4x4 matrix)
            {
                // move forward if reached the maximum in the current matrix array
                if (currArrayIndex >= MaxItemsPerBatch)
                {
                    currListIndex++;
                    currArrayIndex = 0;
                }

                Matrix4x4[] batch;

                if (currListIndex >= batches.Count)
                {
                    batch = new Matrix4x4[MaxItemsPerBatch];
                    batches.Add(batch);
                }
                else
                {
                    batch = batches[currListIndex];
                }

                batch[currArrayIndex++] = matrix;
            }

            /// <summary>
            /// Clears all the matrices.
            /// </summary>
            public void Clear()
            {
                batches.Clear();

                currListIndex = 0;
                currArrayIndex = 0;
            }
        }

        #endregion

        #region Public Attributes

        public const int MaxItemsPerBatch = 1023;

        public Material material;
        public Mesh mesh;

        #endregion

        #region Private Attributes

        private const int MaxColorVariations = 8;
        private static readonly int BaseColorId = Shader.PropertyToID("_BaseColor");

        private MaterialPropertyBlock propertyBlock;
        private List<BatchInfo> batchInfos;

        #endregion

        #region Initialization Methods

        private Batcher()
        {
            propertyBlock = new MaterialPropertyBlock();
            batchInfos = new List<BatchInfo>();
        }

        public Batcher(Mesh mesh, Material material) : this()
        {
            this.mesh = mesh;
            this.material = material;
        }

        #endregion

        #region Methods

        /// <summary>
        /// Adds the given item to be rendered.
        /// </summary>
        /// <param name="color"></param>
        /// <param name="matrix"></param>
        public void AddItem(Color32 color, Matrix4x4 matrix)
        {
            // check if the color was already added to group the matrices
            int foundAtIndex = -1;

            for (int i = 0; i < batchInfos.Count; i++)
            {
                if ((Color)batchInfos[i].color == (Color)color)
                {
                    foundAtIndex = i;
                    break;
                }
            }

            // the color was not added yet
            if (foundAtIndex < 0 && batchInfos.Count >= MaxColorVariations - 1)
            {
                // we have surpassed the limit of colors supported
                Logger.LogWarningFormat("Can not add another color variation to the batcher, because it reached the maximum allowed: {0}. Consider increasing it.", MaxColorVariations.ToString());
                return;
            }
            else if (foundAtIndex < 0 && batchInfos.Count < MaxColorVariations - 1 || foundAtIndex >= 0)
            {
                // color not added yet and still have room for another color, or color added already
                BatchInfo info = foundAtIndex >= 0 ? batchInfos[foundAtIndex] : new BatchInfo(color);
                info.AddMatrix(matrix);

                if (foundAtIndex < 0)
                    batchInfos.Add(info);
            }
        }

        /// <summary>
        /// Draws all the batched stuff.
        /// </summary>
        public void DoRender()
        {
            if (mesh == null || material == null)
                return;

            for (int i = 0; i < batchInfos.Count; i++)
            {
                BatchInfo info = batchInfos[i];
                propertyBlock.SetColor(BaseColorId, info.color);

                List<Matrix4x4[]> batchedMatrices = info.Batches;
                int batches = batchedMatrices.Count;

                for (int j = 0; j < batches; j++)
                {
                    Matrix4x4[] batch = batchedMatrices[j];
                    Graphics.DrawMeshInstanced(mesh, 0, material, batch, batch.Length, propertyBlock, ShadowCastingMode.Off);
                }
            }
        }

        /// <summary>
        /// Clears the data to be rendered.
        /// </summary>
        public void Clear()
        {
            for (int i = batchInfos.Count - 1; i >= 0; i--)
            {
                batchInfos[i].Clear();
                batchInfos.RemoveAt(i);
            }
        }

        #endregion
    }
}