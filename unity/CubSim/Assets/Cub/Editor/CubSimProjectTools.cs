using System;
using System.IO;
using System.Linq;
using UnityEditor;
using UnityEditor.Build;
using UnityEditor.Build.Reporting;
using UnityEditor.SceneManagement;
using UnityEngine;

namespace CubSim.Editor
{
    public static class CubSimProjectTools
    {
        private const string ScenePath = "Assets/Cub/Scenes/Bootstrap.unity";
        private const string UrdfPath = "Assets/StreamingAssets/Robots/mcub.urdf";

        [MenuItem("CubSim/Validate generated mCub URDF")]
        public static void ValidateProject()
        {
            var absoluteUrdfPath = Path.Combine(Application.dataPath, "StreamingAssets/Robots/mcub.urdf");
            if (!File.Exists(absoluteUrdfPath)) throw new FileNotFoundException("Run the model update script first.", absoluteUrdfPath);
            var model = UrdfModel.Parse(File.ReadAllText(absoluteUrdfPath));
            McubUrdfSpawner.ValidateMcubModel(model);

            var scanJoint = model.Joint("base_to_SLC1");
            var scanHead = model.Links["SLC1_link"].Visuals.Single(item => item.Type == UrdfGeometryType.Cylinder);
            if (Mathf.Abs(scanJoint.Origin.PositionRos.z - 0.140f) > 1e-5f)
                throw new InvalidOperationException("Expected provisional SLC1 scan height 0.140 m.");
            if (Mathf.Abs(scanHead.Radius - 0.025f) > 1e-5f)
                throw new InvalidOperationException("Expected SLC1 head radius 0.025 m.");
            if (SlamtecC1Lidar.SamplesPerScan * SlamtecC1Lidar.ScanFrequencyHz != SlamtecC1Lidar.SampleFrequencyHz)
                throw new InvalidOperationException("C1 sample count does not match its sample frequency.");

            CreateBootstrapScene();
            Debug.Log($"CubSim validation passed: {model.Links.Count} links, {model.Joints.Count} joints, C1 5000 samples/s.");
        }

        [MenuItem("CubSim/Create bootstrap scene")]
        public static void CreateBootstrapScene()
        {
            Directory.CreateDirectory(Path.GetDirectoryName(Path.Combine(Application.dataPath, "Cub/Scenes/Bootstrap.unity"))!);
            var scene = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
            new GameObject("SimulationBootstrap").AddComponent<SimulationBootstrap>();
            EditorSceneManager.SaveScene(scene, ScenePath);
            EditorBuildSettings.scenes = new[] { new EditorBuildSettingsScene(ScenePath, true) };
            AssetDatabase.Refresh();
        }

        public static void EnableRos2()
        {
            var target = NamedBuildTarget.Standalone;
            var defines = PlayerSettings.GetScriptingDefineSymbols(target)
                .Split(';', StringSplitOptions.RemoveEmptyEntries)
                .ToList();
            if (!defines.Contains("CUB_ROS2")) defines.Add("CUB_ROS2");
            PlayerSettings.SetScriptingDefineSymbols(target, string.Join(";", defines));
            Debug.Log("Enabled CUB_ROS2 for Standalone builds.");
        }

        public static void DisableRos2()
        {
            var target = NamedBuildTarget.Standalone;
            var defines = PlayerSettings.GetScriptingDefineSymbols(target)
                .Split(';', StringSplitOptions.RemoveEmptyEntries)
                .Where(item => item != "CUB_ROS2");
            PlayerSettings.SetScriptingDefineSymbols(target, string.Join(";", defines));
        }

        public static void BuildWindows()
        {
            ValidateProject();
            if (Directory.Exists("Builds/Windows")) Directory.Delete("Builds/Windows", true);
            Directory.CreateDirectory("Builds/Windows");
            var report = BuildPipeline.BuildPlayer(
                new[] { ScenePath },
                "Builds/Windows/CubSim.exe",
                BuildTarget.StandaloneWindows64,
                BuildOptions.None);
            if (report.summary.result != BuildResult.Succeeded)
                throw new InvalidOperationException($"Unity build failed: {report.summary.result}");
        }
    }

    internal sealed class CubSimStandaloneResources : IPostprocessBuildWithReport
    {
        public int callbackOrder => 10;

        public void OnPostprocessBuild(BuildReport report)
        {
            if (report.summary.platform != BuildTarget.StandaloneWindows64) return;
            var source = Path.Combine(Application.dataPath, "Ros2ForUnity", "Plugins", "Windows", "x86_64", "share");
            if (!Directory.Exists(source))
                throw new DirectoryNotFoundException($"Standalone ROS 2 ament index is missing: {source}");

            var outputDirectory = Path.GetDirectoryName(report.summary.outputPath)!;
            var executableName = Path.GetFileNameWithoutExtension(report.summary.outputPath);
            var destination = Path.Combine(outputDirectory, executableName + "_Data", "Plugins", "x86_64", "share");
            if (Directory.Exists(destination)) Directory.Delete(destination, true);
            foreach (var sourceFile in Directory.GetFiles(source, "*", SearchOption.AllDirectories))
            {
                if (sourceFile.EndsWith(".meta", StringComparison.OrdinalIgnoreCase)) continue;
                var relativePath = Path.GetRelativePath(source, sourceFile);
                var destinationFile = Path.Combine(destination, relativePath);
                Directory.CreateDirectory(Path.GetDirectoryName(destinationFile)!);
                File.Copy(sourceFile, destinationFile, true);
            }
        }
    }
}
