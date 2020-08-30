using UnityEngine;

namespace UnityLibrary
{
    /// <summary>
    /// Simple class that provides singleton functionality to MonoBehaviours. This class might need
    /// a rework to support edge cases.
    /// TODO: Add support to editor, ensure that no other script is attached to the gameObject, when DontDestroyOnLoad is used we might not want to have childs...
    /// </summary>
    /// <typeparam name="T"></typeparam>
    [DisallowMultipleComponent]
    public abstract class MonoBehaviourSingleton<T> : MonoBehaviour where T : MonoBehaviour
    {
        #region Private Attributes

        private static T instance;

        #endregion

        #region Properties

        public static T Instance
        {
            get
            {
                if (instance == null)
                {
                    string objName = string.Format("----- Singleton {0} -----", typeof(T).Name);
                    GameObject obj = new GameObject(objName);
                    instance = obj.AddComponent<T>();
                }

                return instance;
            }
        }

        protected abstract bool DestroyOnLoad { get; }

        #endregion

        #region MonoBehaviour Methods

        protected virtual void Awake()
        {
            if (instance != null && instance != this)
            {
                Destroy(this);
                return;
            }

            instance = this as T;

            if (!DestroyOnLoad)
                DontDestroyOnLoad(instance);
        }

        #endregion
    }
}