using Unity.Mathematics;
using UnityEngine;
using UnityLibrary;

namespace AStar
{
    /// <summary>
    /// Our needed transform information for a node.
    /// </summary>
    public struct NodeTransform
    {
        public readonly float3 pos;
        public readonly float3 fwd;
        public readonly float3 right;
        public readonly float3 up;

        public Vector3 Pos { get { return (Vector3)pos; } }
        public Vector3 Fwd { get { return (Vector3)fwd; } }
        public Vector3 Right { get { return (Vector3)right; } }
        public Vector3 Up { get { return (Vector3)up; } }

        public NodeTransform(float3 pos, float3 hitNormal)
        {
            this.pos = pos;
            this.fwd = float3.zero;
            this.right = float3.zero;
            this.up = float3.zero;

            // the following code computes all axis from the hit normal
            bool normalAproxWorldUp = jobmaths.approxByDotProduct(hitNormal, math.up());

            // if the hit normal matches with the world up just match the world axis
            if (normalAproxWorldUp)
            {
                up = math.up();
                right = math.right();
                fwd = math.forward();

                return;
            }

            // we are considering that objects will be rotated only either in the X axis or Z axis
            bool zCompGreater = math.abs(hitNormal.z) >= math.abs(hitNormal.x);
            up = zCompGreater ? new float3(0.0f, hitNormal.y, hitNormal.z) : new float3(hitNormal.x, hitNormal.y, 0.0f);
            up = math.normalize(up);

            if (zCompGreater)
            {
                right = math.normalize(math.cross(math.up(), up)) * math.sign(hitNormal.z);
                fwd = math.normalize(math.cross(right, up));
            }
            else
            {
                fwd = math.normalize(math.cross(up, math.up())) * math.sign(hitNormal.x);
                right = math.normalize(math.cross(up, fwd));
            }
        }

        #region Methods

        /// <summary>
        /// Gets the rotation of the node.
        /// </summary>
        /// <returns></returns>
        public quaternion GetRotation()
        {
            return quaternion.LookRotation(fwd, up);
        }

        /// <summary>
        /// Gets the given corner point with the given extension, to virtually scale the node.
        /// </summary>
        /// <param name="corner"></param>
        /// <param name="extension"></param>
        /// <param name="localUpwardsFactor"></param>
        /// <returns></returns>
        public float3 GetCorner(NodeRelativeCorner corner, float extension, float localUpwardsFactor, float separationFromMidPoint = 1.0f)
        {
            float3 p = pos;
            extension *= 0.5f;

            switch (corner)
            {
                case NodeRelativeCorner.TopLeft:
                    p += (fwd * separationFromMidPoint - right * separationFromMidPoint) * extension;
                    break;

                case NodeRelativeCorner.TopRight:
                    p += (fwd * separationFromMidPoint + right * separationFromMidPoint) * extension;
                    break;

                case NodeRelativeCorner.BotRight:
                    p += (-fwd * separationFromMidPoint + right * separationFromMidPoint) * extension;
                    break;

                case NodeRelativeCorner.BotLeft:
                    p += (-fwd * separationFromMidPoint - right * separationFromMidPoint) * extension;
                    break;
            }

            p += (localUpwardsFactor * up);

            return p;
        }

        /// <summary>
        /// Gets the direction towards the relative neighbor.
        /// </summary>
        /// <param name="relativeNeighbor"></param>
        /// <returns></returns>
        public float3 GetDirTowardsNeighbor(NodeRelativeNeighbor relativeNeighbor)
        {
            float3 dir = float3.zero;

            switch (relativeNeighbor)
            {
                case NodeRelativeNeighbor.Top:
                    dir = fwd;
                    break;

                case NodeRelativeNeighbor.Right:
                    dir = right;
                    break;

                case NodeRelativeNeighbor.Bottom:
                    dir = -fwd;
                    break;

                case NodeRelativeNeighbor.Left:
                    dir = -right;
                    break;
            }

            return dir;
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

    /// <summary> The information for a given node for the costs & parent in pathfinding. </summary>
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
    /// The possible neighboring layouts for the nodes.
    /// </summary>
    public enum NeighborLayout
    {
        Four = 4
    }

    /// <summary>
    /// The types of nodes that we can have, regarding their walkability.
    /// </summary>
    public enum NodeType : byte
    {
        // Note: the order is important because it is used to simplify a bit some if conditions, so
        // going from less restrictive to more restrictive is the way to go

        Free, // < valid node with floor below, free to walk over
        OccupiedByCharacter, // < with a character over it, walkable but only by the character standing on it
        OccupiedByObstacle, // < an obstacle over it, not walkable
        Invalid // < with no floor below
    }

    /// <summary>
    /// The possible neighboring relative positions of the node.
    /// </summary>
    public enum NodeRelativeNeighbor
    {
        Top,
        Right,
        Bottom,
        Left,

        Count
    }

    /// <summary>
    /// The corners of the node, useful for visual representations.
    /// </summary>
    public enum NodeRelativeCorner
    {
        TopLeft,
        TopRight,
        BotRight,
        BotLeft
    }
}