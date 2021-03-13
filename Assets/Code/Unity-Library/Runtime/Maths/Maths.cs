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