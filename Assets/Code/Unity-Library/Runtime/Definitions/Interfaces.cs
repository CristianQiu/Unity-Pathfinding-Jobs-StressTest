namespace UnityLibrary
{
    #region Pooling

    /// <summary>
    /// Interface used for common objects that can be pooled.
    /// </summary>
    /// <typeparam name="T"></typeparam>
    public interface IPoolable
    {
        /// <summary>
        /// Resets the object attributes when created or returned to the pool.
        /// </summary>
        void Reset();

        /// <summary>
        /// Initializes the object when popped from the pool.
        /// </summary>
        void Init();
    }

    #endregion
}