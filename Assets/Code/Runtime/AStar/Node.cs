using UnityEngine;
using UnityLibrary;

namespace AStar
{
    /// <summary>
    /// The needed transform information for a node.
    /// </summary>
    public struct NodeTransform
    {
        private Vector3 pos;
        private Vector3 fwd;
        private Vector3 right;
        private Vector3 up;

        public Vector3 Pos { get { return pos; } }
        public Vector3 Fwd { get { return fwd; } }
        public Vector3 Right { get { return right; } }
        public Vector3 Up { get { return up; } }

        public NodeTransform(Vector3 pos, Vector3 hitNormal)
        {
            this.pos = pos;
            this.fwd = Vector3.zero;
            this.right = Vector3.zero;
            this.up = Vector3.zero;

            ComputeAllAxisFromHitNormal(hitNormal);
        }

        #region Methods

        /// <summary>
        /// Computes the forward, right and up vectors from the original hit normal.
        /// </summary>
        /// <param name="hitNormal"></param>
        private void ComputeAllAxisFromHitNormal(Vector3 hitNormal)
        {
            bool normalAproxWorldUp = Maths.ApproxByComponents(hitNormal, Vector3.up);

            // if the hit normal matches with the world up just match the world axis
            if (normalAproxWorldUp)
            {
                up = Vector3.up;
                right = Vector3.right;
                fwd = Vector3.forward;

                return;
            }

            // Note: we are considering that objects will be rotated only either in the X axis or Z axis
            bool zCompGreater = Mathf.Abs(hitNormal.z) >= Mathf.Abs(hitNormal.x);

            if (zCompGreater)
                up = new Vector3(0.0f, hitNormal.y, hitNormal.z).normalized;
            else
                up = new Vector3(hitNormal.x, hitNormal.y, 0.0f).normalized;

            if (zCompGreater)
            {
                right = Vector3.Cross(Vector3.up, up).normalized * Mathf.Sign(hitNormal.z);
                fwd = Vector3.Cross(right, up).normalized;
            }
            else
            {
                fwd = Vector3.Cross(up, Vector3.up).normalized * Mathf.Sign(hitNormal.x);
                right = Vector3.Cross(up, fwd).normalized;
            }
        }

        /// <summary>
        /// Gets the rotation of the node.
        /// </summary>
        /// <returns></returns>
        public Quaternion GetRotation()
        {
            return Quaternion.LookRotation(fwd, up);
        }

        #endregion
    }

    /// <summary>
    /// Struct that stores the state of a neighbor node by saving its index and whether is reachable
    /// or not.
    /// </summary>
    public struct NodeNeighbor
    {
        public int neighborIndex;
        public bool isValid;

        public NodeNeighbor(int neighborIndex, bool isValid)
        {
            this.neighborIndex = neighborIndex;
            this.isValid = isValid;
        }
    }

    /// <summary> The information for a given node for the costs & parent in pathfinding </summary>
    public struct NodePathFindInfo
    {
        public int gCost;
        public int hCost;
        public int parentNodeIndex;

        public int FCost { get { return gCost + hCost; } }

        public NodePathFindInfo(int gCost, int hCost, int parentNodeIndex)
        {
            this.gCost = gCost;
            this.hCost = hCost;
            this.parentNodeIndex = parentNodeIndex;
        }
    }

    /// <summary>
    /// The information for a given node for the depth and parent node index in BFS to find
    /// reachable nodes.
    /// </summary>
    public struct NodeBreadthFirstSearchInfo
    {
        public int depth;
        public int parentNodeIndex;

        public NodeBreadthFirstSearchInfo(int depth, int parentNodeIndex)
        {
            this.depth = depth;
            this.parentNodeIndex = parentNodeIndex;
        }
    }

    /// <summary>
    /// The types of nodes that we can have, regarding their walkability.
    /// </summary>
    public enum NodeType
    {
        Free, // < valid node with floor below, free to walk over
        OccupiedByCharacter, // < with a character over it, walkable but only by the character standing on it
        OccupiedByObstacle, // < an obstacle over it, not walkable
        Invalid // < with no floor below
    }

    /// <summary>
    /// The possible neighboring layouts for the nodes.
    /// </summary>
    public enum NeighborLayout
    {
        Four = 4,
        //Eight = 8 // < not implemented at all
    }
}