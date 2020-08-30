using System.Diagnostics;
using UnityEngine;
using Debug = UnityEngine.Debug;

namespace UnityLibrary
{
    /// <summary>
    /// Class to do logging stuff wrapping Unity's Debug.Log system.
    /// </summary>
    public static class Logger
    {
        #region Log Methods

        /// <summary>
        /// Logs a message to the Unity console.
        /// </summary>
        /// <param name="msg"></param>
        [Conditional("DEBUG_LOG")]
        public static void Log(string msg)
        {
            Debug.Log(msg);
        }

        /// <summary>
        /// Logs a message to the Unity console.
        /// </summary>
        /// <param name="msg"></param>
        /// <param name="context"></param>
        [Conditional("DEBUG_LOG")]
        public static void Log(string msg, Object context)
        {
            Debug.Log(msg, context);
        }

        /// <summary>
        /// Logs a formatted message to the Unity console.
        /// </summary>
        /// <param name="format"></param>
        /// <param name="args"></param>
        [Conditional("DEBUG_LOG")]
        public static void LogFormat(string format, params object[] args)
        {
            Debug.LogFormat(format, args);
        }

        /// <summary>
        /// Logs a formatted message to the Unity console.
        /// </summary>
        /// <param name="context"></param>
        /// <param name="format"></param>
        /// <param name="args"></param>
        [Conditional("DEBUG_LOG")]
        public static void LogFormat(Object context, string format, params object[] args)
        {
            Debug.LogFormat(context, format, args);
        }

        #endregion

        #region Warning Methods

        /// <summary>
        /// A variant of Logger.Log that logs a warning message to the Unity console.
        /// </summary>
        /// <param name="msg"></param>
        [Conditional("DEBUG_LOG")]
        public static void LogWarning(string msg)
        {
            Debug.LogWarning(msg);
        }

        /// <summary>
        /// A variant of Logger.Log that logs a warning message to the Unity console.
        /// </summary>
        /// <param name="msg"></param>
        /// <param name="context"></param>
        [Conditional("DEBUG_LOG")]
        public static void LogWarning(string msg, Object context)
        {
            Debug.LogWarning(msg, context);
        }

        /// <summary>
        /// A variant of Logger.LogFormat that logs a formatted warning message to the Unity console.
        /// </summary>
        /// <param name="format"></param>
        /// <param name="args"></param>
        [Conditional("DEBUG_LOG")]
        public static void LogWarningFormat(string format, params object[] args)
        {
            Debug.LogWarningFormat(format, args);
        }

        /// <summary>
        /// A variant of Logger.LogFormat that logs a formatted warning message to the Unity console.
        /// </summary>
        /// <param name="context"></param>
        /// <param name="format"></param>
        /// <param name="args"></param>
        [Conditional("DEBUG_LOG")]
        public static void LogWarningFormat(Object context, string format, params object[] args)
        {
            Debug.LogWarningFormat(context, format, args);
        }

        #endregion

        #region Error Methods

        /// <summary>
        /// A variant of Logger.Log that logs an error message to the Unity console.
        /// </summary>
        /// <param name="msg"></param>
        [Conditional("DEBUG_LOG")]
        public static void LogError(string msg)
        {
            Debug.LogError(msg);
        }

        /// <summary>
        /// A variant of Logger.Log that logs an error message to the Unity console.
        /// </summary>
        /// <param name="msg"></param>
        /// <param name="context"></param>
        [Conditional("DEBUG_LOG")]
        public static void LogError(string msg, Object context)
        {
            Debug.LogError(msg, context);
        }

        /// <summary>
        /// A variant of Logger.LogFormat that logs a formatted error message to the Unity console.
        /// </summary>
        /// <param name="format"></param>
        /// <param name="args"></param>
        [Conditional("DEBUG_LOG")]
        public static void LogErrorFormat(string format, params object[] args)
        {
            Debug.LogErrorFormat(format, args);
        }

        /// <summary>
        /// A variant of Logger.LogFormat that logs a formatted error message to the Unity console.
        /// </summary>
        /// <param name="context"></param>
        /// <param name="format"></param>
        /// <param name="args"></param>
        [Conditional("DEBUG_LOG")]
        public static void LogErrorFormat(Object context, string format, params object[] args)
        {
            Debug.LogErrorFormat(context, format, args);
        }

        #endregion
    }
}