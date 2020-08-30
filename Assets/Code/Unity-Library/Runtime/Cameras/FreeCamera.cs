using UnityEditor;
using UnityEngine;
using UnityEngine.InputSystem;

namespace UnityLibrary
{
    /// <summary>
    /// Simple free camera, mainly for in game debug purposes.
    /// </summary>
    public class FreeCamera : MonoBehaviour
    {
        #region Definitions

        /// <summary>
        /// Class that represents the state of the camera.
        /// </summary>
        private class CameraState
        {
            public float pitch;
            public float roll;
            public float yaw;

            public float x;
            public float y;
            public float z;

            /// <summary>
            /// Sets the camera state from the given transform.
            /// </summary>
            /// <param name="t"></param>
            public void SetFromTransform(Transform t)
            {
                Vector3 eulers = t.eulerAngles;
                Vector3 pos = t.position;

                pitch = eulers.x;
                yaw = eulers.y;
                roll = eulers.z;

                x = pos.x;
                y = pos.y;
                z = pos.z;
            }

            /// <summary>
            /// Updates the given camera transform with this camera state.
            /// </summary>
            /// <param name="t"></param>
            public void UpdateTransform(Transform t)
            {
                t.SetPositionAndRotation(new Vector3(x, y, z), Quaternion.Euler(new Vector3(pitch, yaw, roll)));
            }

            /// <summary>
            /// Translates the camera state by the given translation.
            /// </summary>
            /// <param name="translation"></param>
            public void Translate(Vector3 translation)
            {
                Vector3 rotatedTranslation = Quaternion.Euler(pitch, yaw, roll) * translation;

                x += rotatedTranslation.x;
                y += rotatedTranslation.y;
                z += rotatedTranslation.z;
            }

            /// <summary>
            /// Smoothly interpolate towards the target camera state.
            /// </summary>
            /// <param name="target"></param>
            /// <param name="positionLerpTime"></param>
            /// <param name="rotationLerpTime"></param>
            /// <param name="dt"></param>
            public void LerpTowards(CameraState target, float positionLerpTime, float rotationLerpTime, float dt)
            {
                yaw = Maths.SmoothLerp(yaw, target.yaw, rotationLerpTime, dt);
                pitch = Maths.SmoothLerp(pitch, target.pitch, rotationLerpTime, dt);
                roll = Maths.SmoothLerp(roll, target.roll, rotationLerpTime, dt);

                x = Maths.SmoothLerp(x, target.x, positionLerpTime, dt);
                y = Maths.SmoothLerp(y, target.y, positionLerpTime, dt);
                z = Maths.SmoothLerp(z, target.z, positionLerpTime, dt);
            }
        }

        #endregion

        #region Private Attributes

        [Header("Movement Settings")]
        [Tooltip("Time it takes to interpolate camera position 99% of the way to the target."), Range(0.001f, 1.0f)]
        [SerializeField] private float positionLerpTime = 0.2f;

        [Tooltip("Exponential boost factor on translation, controllable by mouse wheel.")]
        [SerializeField] private float boost = 3.5f;

        [Header("Rotation Settings")]
        [Tooltip("X = Change in mouse position.\nY = Multiplicative factor for camera rotation.")]
        [SerializeField] private AnimationCurve mouseSensitivityCurve = new AnimationCurve(new Keyframe(0.0f, 0.5f, 0.0f, 5.0f), new Keyframe(1.0f, 2.5f, 0.0f, 0.0f));

        [Tooltip("Time it takes to interpolate camera rotation 99% of the way to the target."), Range(0.001f, 1.0f)]
        [SerializeField] private float rotationLerpTime = 0.01f;

        [Tooltip("Whether or not to invert our Y axis for mouse input to rotation.")]
        [SerializeField] private bool invertY = false;

        private readonly CameraState interpolatingCameraState = new CameraState();
        private readonly CameraState targetCameraState = new CameraState();

        #endregion

        #region MonoBehaviour Methods

        private void OnEnable()
        {
            targetCameraState.SetFromTransform(transform);
            interpolatingCameraState.SetFromTransform(transform);
        }

        private void Update()
        {
            float dt = Time.deltaTime;

            if (Keyboard.current.escapeKey.wasPressedThisFrame)
            {
#if UNITY_EDITOR
                EditorApplication.isPlaying = false;
#endif
                Application.Quit();
            }

            if (Mouse.current.rightButton.wasPressedThisFrame)
            {
                Cursor.visible = false;
                Cursor.lockState = CursorLockMode.Locked;
            }

            if (Mouse.current.rightButton.wasReleasedThisFrame)
            {
                Cursor.visible = true;
                Cursor.lockState = CursorLockMode.None;
            }

            // rotation
            if (Mouse.current.rightButton.isPressed)
            {
                Vector2 mouseMovement = Mouse.current.delta.ReadValue() * 0.02f;
                mouseMovement.y = mouseMovement.y * (invertY ? 1 : -1);
                float mouseSensitivityFactor = mouseSensitivityCurve.Evaluate(mouseMovement.magnitude);

                targetCameraState.yaw += mouseMovement.x * mouseSensitivityFactor;
                targetCameraState.pitch += mouseMovement.y * mouseSensitivityFactor;
            }

            // translation
            Vector3 translation = GetInputTranslationDirection() * dt;
            if (Keyboard.current.leftShiftKey.isPressed)
                translation *= 10.0f;

            // modify movement by a boost factor
            boost += Mouse.current.scroll.y.ReadValue() * 0.002f;
            translation *= Mathf.Pow(2.0f, boost);

            targetCameraState.Translate(translation);

            interpolatingCameraState.LerpTowards(targetCameraState, positionLerpTime, rotationLerpTime, dt);
            interpolatingCameraState.UpdateTransform(transform);
        }

        #endregion

        #region Methods

        /// <summary>
        /// Gets the direction of the translation given the keys that are pressed.
        /// </summary>
        /// <returns></returns>
        private Vector3 GetInputTranslationDirection()
        {
            Vector3 direction = new Vector3();

            if (Keyboard.current.wKey.isPressed)
                direction += Vector3.forward;

            if (Keyboard.current.sKey.isPressed)
                direction += Vector3.back;

            if (Keyboard.current.aKey.isPressed)
                direction += Vector3.left;

            if (Keyboard.current.dKey.isPressed)
                direction += Vector3.right;

            if (Keyboard.current.qKey.isPressed)
                direction += Vector3.down;

            if (Keyboard.current.eKey.isPressed)
                direction += Vector3.up;

            return direction;
        }

        #endregion
    }
}