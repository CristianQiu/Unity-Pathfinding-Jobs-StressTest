using System.Collections.Generic;
using UnityEngine;

namespace UnityLibrary
{
    /// <summary> Class that creates and manages a pool of GameObjects preallocated into memory,
    /// given a prefab which is cloned. It is like the Pool<T> class, but cleanly places the
    /// gameObjects in the scene too. </summary>
    public class GameObjectPool
    {
        #region Private Attributes

        private const string PoolsRootName = "----- GameObject Pools -----";

        private const int DefaultSize = 256;
        private const int NumAddedWhenAllowNews = 1;

        private readonly GameObject prefab;

        private readonly Transform poolsRoot;
        private readonly Transform poolRoot;

        private readonly List<GameObject> freedObjs;
        private readonly List<GameObject> usedObjs;

        #endregion

        #region Properties

        public bool AllowNews { get; set; } = true;
        public int NumFreed { get { return freedObjs.Count; } }
        public int NumUsed { get { return usedObjs.Count; } }
        public int NumTotal { get { return freedObjs.Count + usedObjs.Count; } }

        #endregion

        #region Initialization Methods

        public GameObjectPool(GameObject prefab, string pooledObjName) : this(prefab, pooledObjName, DefaultSize, true)
        {
        }

        public GameObjectPool(GameObject prefab, string pooledObjName, int size, bool allowNews)
        {
            this.prefab = prefab;
            AllowNews = allowNews;

            freedObjs = new List<GameObject>(size);
            usedObjs = new List<GameObject>(size);

            // create the pool hosts, first the general one
            poolsRoot = GameObject.Find(PoolsRootName)?.transform;
            if (poolsRoot == null)
                poolsRoot = new GameObject(PoolsRootName).transform;

            // then the specific one
            string poolName = pooledObjName + " Pool";
            poolRoot = new GameObject(poolName).transform;
            poolRoot.SetParent(poolsRoot);

            // prepare the objects
            AddFreeObjects(size);
        }

        /// <summary>
        /// Adds the given quantity of freed gameObjects to the pool.
        /// </summary>
        /// <param name="quantity"></param>
        private void AddFreeObjects(int quantity)
        {
            for (int i = 0; i < quantity; i++)
            {
                GameObject newObj = Object.Instantiate(prefab, Vector3.zero, Quaternion.identity, poolRoot);
                newObj.SetActive(false);
                freedObjs.Add(newObj);
            }
        }

        #endregion

        #region Methods

        /// <summary>
        /// Pops a freed gameObject and moves it to the list of used gameObjects. This is an O(1) operation.
        /// </summary>
        /// <returns></returns>
        public GameObject Pop()
        {
            GameObject obj = null;

            if (freedObjs.Count <= 0)
            {
                if (!AllowNews)
                    Logger.LogWarning("Trying to get a freed gameObject, but the pool is not allowed to grow in size and no more gameObjects are free to return.");
                else
                    AddFreeObjects(NumAddedWhenAllowNews);
            }

            if (freedObjs.Count > 0)
            {
                // get the last one
                int last = freedObjs.Count - 1;
                obj = freedObjs[last];

                // remove it from the freed gameObjects and add it to the used ones
                freedObjs.RemoveAt(last);
                obj.SetActive(true);
                usedObjs.Add(obj);
            }

            return obj;
        }

        /// <summary>
        /// Pushes the given gameObject from the list of used gameObjects and moves it to the list
        /// of freed gameObjects. This is an O(N) operation.
        /// </summary>
        /// <param name="obj"></param>
        public void Push(GameObject obj)
        {
            bool success = usedObjs.Remove(obj);

            if (success)
                Return(obj);
            else
                Logger.LogWarningFormat("Could not find the gameObject that you wanted to free: {0}.", obj.ToString());
        }

        /// <summary>
        /// Pushes all of the gameObjects being used to the pool of freed gameObjects. This is an
        /// O(N) operation.
        /// </summary>
        public void PushAll()
        {
            for (int i = usedObjs.Count - 1; i >= 0; i--)
            {
                GameObject obj = usedObjs[i];

                Return(obj);
                usedObjs.RemoveAt(i);
            }
        }

        /// <summary>
        /// Returns the given gameObject to the pool.
        /// </summary>
        /// <param name="obj"></param>
        private void Return(GameObject obj)
        {
            obj.SetActive(false);
            freedObjs.Add(obj);
        }

        #endregion
    }
}