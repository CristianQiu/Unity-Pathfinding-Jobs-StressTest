namespace UnityLibrary
{
    #region Game Settings

    /// <summary>
    /// The supported languages by the game.
    /// </summary>
    public enum Language
    {
        Spanish,
        English
    }

    #endregion

    #region Time

    /// <summary>
    /// The different types of delta times.
    /// </summary>
    public enum ElapsedTimeType
    {
        ScaledDelta,
        UnscaledDelta
    }

    #endregion

    #region Maths

    /// <summary>
    /// The different types of interpolations for tweens.
    /// </summary>
    public enum InterpolationType
    {
        AnimationCurve,
        Linear,
        Exponential
    }

    #endregion

    #region UI

    /// <summary>
    /// The possible fading states.
    /// </summary>
    public enum FadeState
    {
        Invalid = -1,

        FadingIn,
        FadedIn,
        FadingOut,
        FadedOut
    }

    /// <summary>
    /// The possible menu screens of the game.
    /// </summary>
    public enum MenuScreenType
    {
        Startup,
        Menu,
        Loading,

        Count
    }

    /// <summary>
    /// The possible in game screens of the game.
    /// </summary>
    public enum InGameScreenType
    {
        WorldUI,
        Empty,
        HUD,

        Count
    }

    #endregion
}