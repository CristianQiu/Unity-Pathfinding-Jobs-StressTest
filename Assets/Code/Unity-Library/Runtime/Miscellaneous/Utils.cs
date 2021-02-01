using UnityEngine;

namespace UnityLibrary
{
    /// <summary>
    /// Class providing useful methods.
    /// </summary>
    public static class Utils
    {
        #region Unity Hierarchy / Components Methods

        /// <summary>
        /// Tries to get the given component from the gameObject. If it does not have it, adds the
        /// component before returning it.
        /// </summary>
        /// <typeparam name="T"></typeparam>
        /// <param name="gameObject"></param>
        /// <param name="hadToCreateIt"></param>
        /// <returns></returns>
        public static T GetOrAddComponent<T>(GameObject gameObject, out bool hadToCreateIt) where T : Component
        {
            hadToCreateIt = false;
            bool alreadyHasIt = gameObject.TryGetComponent(out T component);

            if (!alreadyHasIt)
            {
                component = gameObject.AddComponent<T>();
                hadToCreateIt = true;
            }

            return component;
        }

        /// <summary>
        /// Tries to get the given component from the transform. If it does not have it, adds the
        /// component before returning it.
        /// </summary>
        /// <typeparam name="T"></typeparam>
        /// <param name="transform"></param>
        /// <param name="hadToCreateIt"></param>
        /// <returns></returns>
        public static T GetOrAddComponent<T>(Transform transform, out bool hadToCreateIt) where T : Component
        {
            return GetOrAddComponent<T>(transform.gameObject, out hadToCreateIt);
        }

        #endregion
    }
}