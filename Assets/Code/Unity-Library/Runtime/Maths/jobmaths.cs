using Unity.Mathematics;

namespace UnityLibrary
{
    /// <summary>
    /// Helper class to do stuff related with jobs that Unity's math class does not include. Notice
    /// that I have followed the naming convention from the math class by mainly removing the
    /// initial capital letter.
    /// </summary>
    public static class jobmaths
    {
        #region Public Attributes

        public const float GREATER_EPSILON = 0.0001f;

        #endregion

        #region Constants Methods

        // Note: These could be static readonly but for some reason math.forward(), math.right(),
        // etc... are functions. There may be a reason for it so I am just copying Unity's standards
        // here, just like I did with naming convention.

        /// <summary>
        /// Positive infinity float3.
        /// </summary>
        /// <returns></returns>
        public static float3 positiveInfinity()
        {
            return new float3(math.INFINITY, math.INFINITY, math.INFINITY);
        }

        /// <summary>
        /// Negative infinity float3.
        /// </summary>
        /// <returns></returns>
        public static float3 negativeInfinity()
        {
            return new float3(float.NegativeInfinity, float.NegativeInfinity, float.NegativeInfinity);
        }

        #endregion

        #region Distance Methods

        /// <summary>
        /// Gets whether two numbers are approximately the same given the threshold.
        /// </summary>
        /// <param name="a"></param>
        /// <param name="b"></param>
        /// <param name="includedThreshold"></param>
        /// <returns></returns>
        public static bool approx(float a, float b, float includedThreshold = GREATER_EPSILON)
        {
            return math.distance(a, b) <= includedThreshold;
        }

        /// <summary>
        /// Gets whether two vectors are approximately equal depending on the given threshold, using
        /// the dot product.
        /// </summary>
        /// <param name="a"></param>
        /// <param name="b"></param>
        /// <param name="includedThreshold"></param>
        /// <returns></returns>
        public static bool approxByDotProduct(float3 a, float3 b, float includedThreshold = GREATER_EPSILON)
        {
            float dot = math.dot(a, b);

            return approx(dot, 1.0f, includedThreshold);
        }

        #endregion
    }
}