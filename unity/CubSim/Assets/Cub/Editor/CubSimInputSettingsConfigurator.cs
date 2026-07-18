using UnityEditor;
using UnityEngine;
using UnityEngine.InputSystem;

namespace CubSim.Editor
{
    internal static class CubSimInputSettingsConfigurator
    {
        private const string SettingsPath = "Assets/Cub/CubSimInputSettings.asset";

        [InitializeOnLoadMethod]
        private static void Configure()
        {
            var settings = AssetDatabase.LoadAssetAtPath<InputSettings>(SettingsPath);
            if (settings == null)
            {
                settings = ScriptableObject.CreateInstance<InputSettings>();
                AssetDatabase.CreateAsset(settings, SettingsPath);
            }

            var changed = false;
            if (settings.backgroundBehavior != InputSettings.BackgroundBehavior.IgnoreFocus)
            {
                settings.backgroundBehavior = InputSettings.BackgroundBehavior.IgnoreFocus;
                changed = true;
            }
            if (settings.editorInputBehaviorInPlayMode !=
                InputSettings.EditorInputBehaviorInPlayMode.AllDeviceInputAlwaysGoesToGameView)
            {
                settings.editorInputBehaviorInPlayMode =
                    InputSettings.EditorInputBehaviorInPlayMode.AllDeviceInputAlwaysGoesToGameView;
                changed = true;
            }

            if (changed)
            {
                EditorUtility.SetDirty(settings);
                AssetDatabase.SaveAssets();
            }
            if (InputSystem.settings != settings)
            {
                InputSystem.settings = settings;
            }
            PlayerSettings.runInBackground = true;
        }
    }
}
