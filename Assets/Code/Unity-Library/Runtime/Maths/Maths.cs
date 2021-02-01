using UnityEngine;

namespace UnityLibrary
{
    /// <summary>
    /// Supporting class to Unity's Mathf library.
    /// </summary>
    public static class Maths
    {
        #region Public Attributes

        public const float GreaterEpsilon = 0.0001f;

        #endregion

        #region Distance Methods

        /// <summary>
        /// Gets whether two vectors are approximately equal depending on the given threshold, for
        /// each of its components.
        /// </summary>
        /// <param name="a"></param>
        /// <param name="b"></param>
        /// <param name="includedThreshold"></param>
        /// <returns></returns>
        public static bool ApproxByComponents(Vector3 a, Vector3 b, float includedThreshold = GreaterEpsilon)
        {
            bool approxX = Approx(a.x, b.x, includedThreshold);
            bool approxY = Approx(a.y, b.y, includedThreshold);
            bool approxZ = Approx(a.z, b.z, includedThreshold);

            return approxX && approxY && approxZ;
        }

        /// <summary>
        /// Gets whether two numbers are approximately equal depending on the given threshold.
        /// </summary>
        /// <param name="a"></param>
        /// <param name="b"></param>
        public static bool Approx(float a, float b, float includedThreshold = GreaterEpsilon)
        {
            return Dist(a, b) <= includedThreshold;
        }

        /// <summary>
        /// Gets the distance between two numbers. These can be be positive or negative.
        /// </summary>
        /// <param name="a"></param>
        /// <param name="b"></param>
        public static float Dist(float a, float b)
        {
            return Mathf.Abs(a - b);
        }

        #endregion

        #region Lerp \ Remap Methods

        /// <summary>
        /// Framerate independant smooth lerping.
        /// </summary>
        /// <param name="currValue"></param>
        /// <param name="targetValue"></param>
        /// <param name="lerpTime"></param>
        /// Time it takes to interpolate value 99% of the way to the target.
        /// <param name="dt"></param>
        /// <returns></returns>
        public static float SmoothLerp(float currValue, float targetValue, float lerpTime, float dt)
        {
            float t = SmoothLerpPercentage(lerpTime, dt);

            return Mathf.Lerp(currValue, targetValue, t);
        }

        /// <summary>
        /// Calculates the lerp amount, such that we get 99% of the way to our target in the
        /// specified time.
        /// </summary>
        /// <param name="lerpTime"></param>
        /// <param name="dt"></param>
        /// <returns></returns>
        private static float SmoothLerpPercentage(float lerpTime, float dt)
        {
            return 1.0f - Mathf.Exp((Mathf.Log(1.0f - 0.99f) / lerpTime) * dt);
        }

        #endregion
    }
}