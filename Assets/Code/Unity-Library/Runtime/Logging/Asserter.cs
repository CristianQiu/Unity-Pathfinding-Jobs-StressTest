using System.Diagnostics;
using UnityEngine;
using Debug = UnityEngine.Debug;

namespace UnityLibrary
{
    /// <summary>
    /// Class to do asserting stuff wrapping Unity's Debug.Assert system. https://docs.unity3d.com/ScriptReference/Debug.Assert.html
    /// </summary>
    public static class Asserter
    {
        #region Methods

        /// <summary>
        /// Asserts that a certain condition is met and sends a message to the Unity console if the
        /// condition is not met.
        /// </summary>
        /// <param name="condition"></param>
        /// <param name="onFailMsg"></param>
        [Conditional("UNITY_ASSERTIONS")]
        public static void Assert(bool condition, string onFailMsg)
        {
            Debug.Assert(condition, onFailMsg);
        }

        /// <summary>
        /// Asserts that a certain condition is met and sends a message to the Unity console if the
        /// condition is not met.
        /// </summary>
        /// <param name="condition"></param>
        /// <param name="onFailMsg"></param>
        /// <param name="context"></param>
        [Conditional("UNITY_ASSERTIONS")]
        public static void Assert(bool condition, string onFailMsg, Object context)
        {
            Debug.Assert(condition, onFailMsg, context);
        }

        /// <summary>
        /// Asserts that a certain condition is met and sends a formatted message to the Unity
        /// console if the condition is not met.
        /// </summary>
        /// <param name="condition"></param>
        /// <param name="onFailMsgFormat"></param>
        /// <param name="args"></param>
        [Conditional("UNITY_ASSERTIONS")]
        public static void AssertFormat(bool condition, string onFailMsgFormat, params object[] args)
        {
            Debug.AssertFormat(condition, onFailMsgFormat, args);
        }

        /// <summary>
        /// Asserts that a certain condition is met and sends a formatted message to the Unity
        /// console if the condition is not met.
        /// </summary>
        /// <param name="condition"></param>
        /// <param name="context"></param>
        /// <param name="onFailMsgFormat"></param>
        /// <param name="args"></param>
        [Conditional("UNITY_ASSERTIONS")]
        public static void AssertFormat(bool condition, Object context, string onFailMsgFormat, params object[] args)
        {
            Debug.AssertFormat(condition, context, onFailMsgFormat, args);
        }

        #endregion
    }
}