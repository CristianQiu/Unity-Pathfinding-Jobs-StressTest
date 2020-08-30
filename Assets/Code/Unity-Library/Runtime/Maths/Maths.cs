using UnityEngine;

namespace UnityLibrary
{
    /// <summary>
    /// Supporting class to Unity's Mathf library.
    /// </summary>
    public static class Maths
    {
        #region Public Attributes

        public const float EpsilonDist = 0.0001f;

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
        public static bool ApproxByComponents(Vector3 a, Vector3 b, float includedThreshold = EpsilonDist)
        {
            bool approxX = Approx(a.x, b.x, includedThreshold);
            bool approxY = Approx(a.y, b.y, includedThreshold);
            bool approxZ = Approx(a.z, b.z, includedThreshold);

            return approxX && approxY && approxZ;
        }

        /// <summary>
        /// Gets whether two vectors are approximately equal depending on the given threshold, using
        /// the dot product.
        /// </summary>
        /// <param name="a"></param>
        /// <param name="b"></param>
        /// <param name="includedThreshold"></param>
        /// <returns></returns>
        public static bool ApproxByDotProduct(Vector3 a, Vector3 b, float includedThreshold = EpsilonDist)
        {
            float dot = Vector3.Dot(a, b);

            return Approx(dot, 1.0f, includedThreshold);
        }

        /// <summary>
        /// Gets whether two numbers are approximately equal depending on the given threshold.
        /// </summary>
        /// <param name="a"></param>
        /// <param name="b"></param>
        public static bool Approx(float a, float b, float includedThreshold = EpsilonDist)
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
        /// Remaps a value within the origin range to the destiny range.
        /// </summary>
        /// <param name="origVal"></param>
        /// <param name="origA"></param>
        /// <param name="origB"></param>
        /// <param name="destA"></param>
        /// <param name="destB"></param>
        /// <returns></returns>
        public static float Remap(float origVal, float origA, float origB, float destA, float destB)
        {
            float t = Mathf.InverseLerp(origA, origB, origVal);

            return Mathf.Lerp(destA, destB, t);
        }

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
        /// Framerate independant smooth lerping.
        /// </summary>
        /// <param name="currValue"></param>
        /// <param name="targetValue"></param>
        /// <param name="lerpTime"></param>
        /// Time it takes to interpolate value 99% of the way to the target.
        /// <param name="dt"></param>
        /// <returns></returns>
        public static Vector3 SmoothLerp(Vector3 currValue, Vector3 targetValue, float lerpTime, float dt)
        {
            float t = SmoothLerpPercentage(lerpTime, dt);

            return Vector3.Lerp(currValue, targetValue, t);
        }

        /// <summary>
        /// Framerate independant smooth lerping.
        /// </summary>
        /// <param name="currValue"></param>
        /// <param name="targetValue"></param>
        /// <param name="lerpTime"></param>
        /// Time it takes to interpolate value 99% of the way to the target.
        /// <param name="dt"></param>
        /// <returns></returns>
        public static Quaternion SmoothSlerp(Quaternion currValue, Quaternion targetValue, float lerpTime, float dt)
        {
            float t = SmoothLerpPercentage(lerpTime, dt);

            return Quaternion.Slerp(currValue, targetValue, t);
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

        #region Bezier Methods

        /// <summary>
        /// Quadratic bezier interpolation.
        /// </summary>
        /// <param name="p0"></param>
        /// <param name="pControl"></param>
        /// <param name="p2"></param>
        /// <param name="t"></param>
        /// <returns></returns>
        public static Vector3 QuadraticBezierInterpolation(Vector3 p0, Vector3 pControl, Vector3 p2, float t)
        {
            float invT = 1.0f - t;

            return invT * invT * p0 + 2.0f * invT * t * pControl + t * t * p2;
        }

        #endregion

        #region Line Methods

        /// http://wiki.unity3d.com/index.php/3d_Math_functions?_ga=2.91029276.781375376.1596710751-1954184310.1596710751

        /// <summary>
        /// Two non-parallel lines which may or may not touch each other have a point on each line
        /// which are closest to each other. This function finds those two points. If the lines are
        /// not parallel, returns true, otherwise false.
        /// </summary>
        /// <param name="closestPointOnLine1"></param>
        /// <param name="closestPointOnLine2"></param>
        /// <param name="lineDir1"></param>
        /// <param name="pointFromLine1"></param>
        /// <param name="lineDir2"></param>
        /// <param name="pointFromLine2"></param>
        /// <returns></returns>
        public static bool ClosestPointsOnTwoLines(out Vector3 closestPointOnLine1, out Vector3 closestPointOnLine2, Vector3 lineDir1, Vector3 pointFromLine1, Vector3 lineDir2, Vector3 pointFromLine2)
        {
            closestPointOnLine1 = Vector3.zero;
            closestPointOnLine2 = Vector3.zero;

            float a = 1.0f;
            float b = Vector3.Dot(lineDir1, lineDir2);
            float e = 1.0f;

            float d = a * e - b * b;

            bool areParallel = Approx(d, 0.0f);

            if (!areParallel)
            {
                Vector3 r = pointFromLine1 - pointFromLine2;

                float c = Vector3.Dot(lineDir1, r);
                float f = Vector3.Dot(lineDir2, r);

                float s = (b * f - c * e) / d;
                float t = (a * f - c * b) / d;

                closestPointOnLine1 = pointFromLine1 + lineDir1 * s;
                closestPointOnLine2 = pointFromLine2 + lineDir2 * t;
            }

            return !areParallel;
        }

        #endregion
    }
}