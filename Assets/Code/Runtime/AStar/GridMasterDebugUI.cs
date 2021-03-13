using UnityEngine;
using UnityEngine.UI;

namespace AStar
{
    /// <summary>
    /// Class providing a small UI module to display or change useful settings while developing the game.
    /// </summary>
    public class GridMasterDebugUI : MonoBehaviour
    {
#if DEBUG_RENDER

        #region Private Attributes

        [Header("Buttons")]
        [SerializeField] private Button buttonCreateGrid = null;

        [Header("Toggles")]
        [SerializeField] private Toggle toggleShowNodes = null;
        [SerializeField] private Toggle toggleShowNodesConnections = null;

        #endregion

        #region MonoBehaviour Methods

        private void Start()
        {
            toggleShowNodes.SetIsOnWithoutNotify(GridMasterDebug.Instance.ShowNodes);
            toggleShowNodesConnections.SetIsOnWithoutNotify(GridMasterDebug.Instance.ShowConnections);

            buttonCreateGrid.onClick.AddListener(OnClickCreateGrid);
            toggleShowNodes.onValueChanged.AddListener(OnToggleShowNodes);
            toggleShowNodesConnections.onValueChanged.AddListener(OnToggleShowNodesConnections);
        }

        private void OnDestroy()
        {
            buttonCreateGrid.onClick.RemoveListener(OnClickCreateGrid);
            toggleShowNodes.onValueChanged.RemoveListener(OnToggleShowNodes);
            toggleShowNodesConnections.onValueChanged.RemoveListener(OnToggleShowNodesConnections);
        }

        #endregion

        #region Callbacks

        /// <summary>
        /// Called when the user clicks the create grid button.
        /// </summary>
        private void OnClickCreateGrid()
        {
            GridMaster.Instance.CreateGrid();
        }

        /// <summary>
        /// Called when the user toggles showing the nodes.
        /// </summary>
        /// <param name="on"></param>
        private void OnToggleShowNodes(bool on)
        {
            GridMasterDebug.Instance.ShowNodes = on;
        }

        /// <summary>
        /// Called when the user toggles the show connections.
        /// </summary>
        /// <param name="on"></param>
        private void OnToggleShowNodesConnections(bool on)
        {
            GridMasterDebug.Instance.ShowConnections = on;
        }

        #endregion
    }

#endif
}