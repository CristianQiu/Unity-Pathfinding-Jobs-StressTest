using UnityEngine;
using UnityEngine.UI;

namespace AStar
{
    /// <summary>
    /// Class providing a small UI module to display or change useful settings while developing the game.
    /// </summary>
    public class GridMasterDebugUI : MonoBehaviour
    {
        #region Private Attributes

        [Header("Toggles")]
        [SerializeField] private Toggle toggleShowNodes = null;
        [SerializeField] private Toggle toggleShowNodesConnections = null;

        #endregion

        #region Properties

        #endregion

        #region MonoBehaviour Methods

        private void Start()
        {
#if DEBUG_RENDER
            toggleShowNodes.SetIsOnWithoutNotify(GridMaster.Instance.ShowNodes);
            toggleShowNodesConnections.SetIsOnWithoutNotify(GridMaster.Instance.ShowNodesConnections);
#else
            Destroy(gameObject);
#endif
        }

        #endregion

        #region Callbacks

        /// <summary>
        /// Called when the user clicks the create grid button.
        /// </summary>
        public void OnClickCreateGrid()
        {
            GridMaster.Instance.CreateGrid();
        }

        /// <summary>
        /// Called when the user toggles showing the nodes.
        /// </summary>
        /// <param name="on"></param>
        public void OnToggleShowNodes(bool on)
        {
#if DEBUG_RENDER
            GridMaster.Instance.ShowNodes = on;
#endif
        }

        /// <summary>
        /// Called when the user toggles the show connections.
        /// </summary>
        /// <param name="on"></param>
        public void OnToggleShowNodesConnections(bool on)
        {
#if DEBUG_RENDER
            GridMaster.Instance.ShowNodesConnections = on;
#endif
        }

        #endregion
    }
}