using System.Collections.Generic;

namespace UnityLibrary
{
    /// <summary>
    /// Class that creates and manages a pool of common objects preallocated into memory.
    /// </summary>
    /// <typeparam name="T"></typeparam>
    public class Pool<T> where T : class, IPoolable, new()
    {
        #region Private Attributes

        private const int DefaultSize = 256;
        private const int NumAddedWhenAllowNews = 1;

        private readonly List<T> freedObjs;
        private readonly List<T> usedObjs;

        #endregion

        #region Properties

        public bool AllowNews { get; set; } = true;
        public int NumFreed { get { return freedObjs.Count; } }
        public int NumUsed { get { return usedObjs.Count; } }
        public int NumTotal { get { return freedObjs.Count + usedObjs.Count; } }

        #endregion

        #region Initialization Methods

        public Pool() : this(DefaultSize, true)
        {
        }

        public Pool(int size, bool allowNews)
        {
            AllowNews = allowNews;

            freedObjs = new List<T>(size);
            usedObjs = new List<T>(size);

            // prepare the objects
            AddFreeObjects(size);
        }

        /// <summary>
        /// Adds the given quantity of freed objects to the pool.
        /// </summary>
        /// <param name="quantity"></param>
        private void AddFreeObjects(int quantity)
        {
            for (int i = 0; i < quantity; i++)
            {
                T newObj = new T();
                newObj.Reset();
                freedObjs.Add(newObj);
            }
        }

        #endregion

        #region Methods

        /// <summary>
        /// Pops a freed object and moves it to the list of used objects. This is an O(1) operation.
        /// </summary>
        /// <returns></returns>
        public T Pop()
        {
            T obj = null;

            if (freedObjs.Count <= 0)
            {
                if (!AllowNews)
                    Logger.LogWarning("Trying to get a freed object, but the pool is not allowed to grow in size and no more objects are free to return.");
                else
                    AddFreeObjects(NumAddedWhenAllowNews);
            }

            if (freedObjs.Count > 0)
            {
                // get the last one and initialize it
                int last = freedObjs.Count - 1;
                obj = freedObjs[last];

                // remove it from the freed objects and add it to the used ones
                freedObjs.RemoveAt(last);
                usedObjs.Add(obj);

                obj.Init();
            }

            return obj;
        }

        /// <summary>
        /// Pushes the given object from the list of used objects and moves it to the list of freed
        /// objects. This is an O(N) operation.
        /// </summary>
        /// <param name="obj"></param>
        public void Push(T obj)
        {
            bool success = usedObjs.Remove(obj);

            if (success)
            {
                // reset before it is added to the list of free objs
                obj.Reset();
                freedObjs.Add(obj);
            }
            else
            {
                Logger.LogWarningFormat("Could not find the object that you wanted to free: {0}.", obj.ToString());
            }
        }

        /// <summary>
        /// Pushes all of the objects being used to the pool of freed objects. This is an O(N) operation.
        /// </summary>
        public void PushAll()
        {
            for (int i = usedObjs.Count - 1; i >= 0; i--)
            {
                T obj = usedObjs[i];
                obj.Reset();

                usedObjs.RemoveAt(i);
                freedObjs.Add(obj);
            }
        }

        #endregion
    }
}