using System;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Xml.Linq;
using Unity.Mathematics;
using Unity.Robotics.UrdfImporter;
using UnityEditor;
using UnityEngine;
using UnitySensors.DataType.LiDAR;
using UnitySensors.Sensor.LiDAR;

namespace CubSim.Editor
{
    public static class McubPrefabImporter
    {
        public const string UrdfAssetPath = "Assets/StreamingAssets/Robots/mcub.urdf";
        public const string ImportUrdfAssetPath = "Assets/Cub/Generated/Import/mcub.urdf";
        public const string PrefabAssetPath = "Assets/Cub/Generated/Resources/mcub.prefab";
        public const string ScanPatternAssetPath = "Assets/Cub/Generated/Resources/SlamtecC1ScanPattern.asset";

        private static readonly XNamespace CubSimNamespace =
            "https://github.com/tomoki-na/Cub_ROS/cubsim";

        [MenuItem("CubSim/Import mCub URDF prefab")]
        public static GameObject Import()
        {
            var absoluteUrdfPath = Path.GetFullPath(UrdfAssetPath);
            if (!File.Exists(absoluteUrdfPath))
            {
                throw new FileNotFoundException(
                    "Generated mCub URDF is missing. Run the model update script first.",
                    absoluteUrdfPath);
            }

            var document = XDocument.Load(absoluteUrdfPath);
            var sensorConfig = ReadSensorConfig(document);
            var wheelRadius = ReadWheelRadius(document, "left_wheel_link");
            var wheelTrack = ReadWheelTrack(document);
            var wheelEffort = ReadWheelEffort(document);
            var scanPattern = CreateOrUpdateScanPattern(sensorConfig);

            Directory.CreateDirectory(Path.GetDirectoryName(ImportUrdfAssetPath)!);
            var importUrdfPath = Path.GetFullPath(ImportUrdfAssetPath);
            File.Copy(absoluteUrdfPath, importUrdfPath, true);
            AssetDatabase.ImportAsset(ImportUrdfAssetPath, ImportAssetOptions.ForceSynchronousImport);
            Directory.CreateDirectory(Path.GetDirectoryName(PrefabAssetPath)!);
            Selection.activeObject = null;
            var settings = ImportSettings.DefaultSettings();
            settings.chosenAxis = ImportSettings.axisType.yAxis;
            settings.convexMethod = ImportSettings.convexDecomposer.unity;
            settings.OverwriteExistingPrefabs = true;

            GameObject imported = null;
            var importRoutine = UrdfRobotExtensions.Create(importUrdfPath, settings);
            while (importRoutine.MoveNext())
            {
                if (importRoutine.Current != null)
                {
                    imported = importRoutine.Current;
                }
            }

            if (imported == null)
            {
                throw new InvalidOperationException("URDF-Importer did not create an mCub GameObject.");
            }

            try
            {
                RemoveUrdfImporterController(imported);
                ConvertMaterialsToActivePipeline(imported);
                SetLayerRecursively(imported, Physics.IgnoreRaycastLayer);

                var baseLink = FindRequired(imported.transform, "base_link");
                var leftWheel = FindRequired(imported.transform, "left_wheel_link");
                var rightWheel = FindRequired(imported.transform, "right_wheel_link");
                var sensorLink = FindRequired(imported.transform, sensorConfig.Link);

                var rootBody = baseLink.GetComponent<ArticulationBody>();
                var leftBody = leftWheel.GetComponent<ArticulationBody>();
                var rightBody = rightWheel.GetComponent<ArticulationBody>();
                if (rootBody == null || leftBody == null || rightBody == null)
                {
                    throw new InvalidOperationException(
                        "URDF-Importer did not create the required ArticulationBody hierarchy.");
                }

                rootBody.immovable = false;
                rootBody.useGravity = true;
                rootBody.linearDamping = 0.05f;
                rootBody.angularDamping = 0.05f;

                var robot = baseLink.gameObject.AddComponent<McubRobot>();
                robot.ConfigureArticulation(rootBody, leftBody, rightBody, wheelRadius, wheelTrack, wheelEffort);

                var lidar = sensorLink.gameObject.AddComponent<RaycastLiDARSensor>();
                ConfigureLidar(lidar, scanPattern, sensorConfig);
                sensorLink.gameObject.AddComponent<UnitySensorsC1Lidar>();
                baseLink.gameObject.AddComponent<McubRos2Bridge>();

                var prefab = PrefabUtility.SaveAsPrefabAsset(imported, PrefabAssetPath, out var success);
                if (!success || prefab == null)
                {
                    throw new InvalidOperationException("Failed to save the generated mCub prefab.");
                }

                AssetDatabase.SaveAssets();
                Debug.Log(
                    $"Imported mCub with URDF-Importer: {PrefabAssetPath}; " +
                    $"wheel radius={wheelRadius:F3} m, track={wheelTrack:F3} m, " +
                    $"C1={sensorConfig.Samples} points at {sensorConfig.Frequency:F1} Hz.");
                return prefab;
            }
            finally
            {
                UnityEngine.Object.DestroyImmediate(imported);
                Selection.activeObject = null;
            }
        }

        public static GameObject LoadAndValidateGeneratedPrefab()
        {
            var prefab = AssetDatabase.LoadAssetAtPath<GameObject>(PrefabAssetPath);
            if (prefab == null)
            {
                throw new FileNotFoundException("Generated mCub prefab is missing.", PrefabAssetPath);
            }

            var robot = prefab.GetComponentInChildren<McubRobot>(true);
            var lidar = prefab.GetComponentInChildren<UnitySensorsC1Lidar>(true);
            var sensor = prefab.GetComponentInChildren<RaycastLiDARSensor>(true);
            if (robot == null || robot.RootArticulation == null || robot.MaximumMotorTorque <= 0f)
            {
                throw new InvalidOperationException("Generated mCub prefab has no configured ArticulationBody controller.");
            }

            if (lidar == null || sensor == null || sensor.pointsNum != 500)
            {
                throw new InvalidOperationException("Generated mCub prefab has no configured UnitySensors C1.");
            }

            var casterSwivel = FindRequired(prefab.transform, "caster_swivel_link");
            var casterWheel = FindRequired(prefab.transform, "caster_wheel_link");
            if (casterSwivel.GetComponent<ArticulationBody>()?.jointType != ArticulationJointType.FixedJoint ||
                casterWheel.GetComponent<ArticulationBody>()?.jointType != ArticulationJointType.FixedJoint ||
                casterWheel.GetComponentInChildren<SphereCollider>(true) == null)
            {
                throw new InvalidOperationException("Generated mCub prefab has no stable ball-caster approximation.");
            }

            return prefab;
        }

        private static void RemoveUrdfImporterController(GameObject imported)
        {
            foreach (var controller in imported.GetComponentsInChildren<
                         Unity.Robotics.UrdfImporter.Control.Controller>(true))
            {
                UnityEngine.Object.DestroyImmediate(controller);
            }
        }

        private static void ConvertMaterialsToActivePipeline(GameObject imported)
        {
            var urpShader = Shader.Find("Universal Render Pipeline/Lit");
            if (urpShader == null)
            {
                return;
            }

            foreach (var material in imported.GetComponentsInChildren<Renderer>(true)
                         .SelectMany(renderer => renderer.sharedMaterials)
                         .Where(material => material != null)
                         .Distinct())
            {
                var color = material.HasProperty("_Color") ? material.color : Color.white;
                material.shader = urpShader;
                if (material.HasProperty("_BaseColor"))
                {
                    material.SetColor("_BaseColor", color);
                }

                EditorUtility.SetDirty(material);
            }
        }

        private static void ConfigureLidar(
            RaycastLiDARSensor sensor,
            ScanPattern scanPattern,
            SensorConfig config)
        {
            var serialized = new SerializedObject(sensor);
            SetObject(serialized, "_scanPattern", scanPattern);
            SetInteger(serialized, "_pointsNumPerScan", config.Samples);
            SetFloat(serialized, "_minRange", config.MinRange);
            SetFloat(serialized, "_maxRange", config.MaxRange);
            SetFloat(serialized, "_gaussianNoiseSigma", 0f);
            SetFloat(serialized, "_maxIntensity", 255f);
            SetInteger(serialized, "_raycastLayerMask", Physics.DefaultRaycastLayers);
            SetFloat(serialized, "_frequency", config.Frequency);
            serialized.ApplyModifiedPropertiesWithoutUndo();
        }

        private static ScanPattern CreateOrUpdateScanPattern(SensorConfig config)
        {
            Directory.CreateDirectory(Path.GetDirectoryName(ScanPatternAssetPath)!);
            var pattern = AssetDatabase.LoadAssetAtPath<ScanPattern>(ScanPatternAssetPath);
            if (pattern == null)
            {
                pattern = ScriptableObject.CreateInstance<ScanPattern>();
                AssetDatabase.CreateAsset(pattern, ScanPatternAssetPath);
            }

            var angleIncrement = Mathf.PI * 2f / config.Samples;
            pattern.scans = new float3[config.Samples];
            for (var index = 0; index < config.Samples; index++)
            {
                var angle = -Mathf.PI + index * angleIncrement;
                pattern.scans[index] = new float3(-Mathf.Sin(angle), 0f, Mathf.Cos(angle));
            }

            pattern.size = config.Samples;
            pattern.minZenithAngle = 0f;
            pattern.maxZenithAngle = 0f;
            pattern.minAzimuthAngle = -180f;
            pattern.maxAzimuthAngle = 180f - 360f / config.Samples;
            EditorUtility.SetDirty(pattern);
            return pattern;
        }

        private static SensorConfig ReadSensorConfig(XDocument document)
        {
            var sensor = document.Root?.Element(CubSimNamespace + "sensor")
                         ?? throw new InvalidOperationException("mCub URDF has no CubSim sensor metadata.");
            if ((string)sensor.Attribute("type") != "raycast_lidar")
            {
                throw new InvalidOperationException("Only the UnitySensors raycast_lidar profile is supported for mCub.");
            }

            return new SensorConfig(
                Required(sensor, "link"),
                ParseInt(sensor, "samples"),
                ParseFloat(sensor, "frequency"),
                ParseFloat(sensor, "min_range"),
                ParseFloat(sensor, "max_range"));
        }

        private static float ReadWheelRadius(XDocument document, string linkName)
        {
            var link = document.Root?.Elements("link")
                .SingleOrDefault(item => (string)item.Attribute("name") == linkName)
                ?? throw new InvalidOperationException($"mCub URDF has no link '{linkName}'.");
            var radius = link.Descendants("cylinder").FirstOrDefault()?.Attribute("radius")?.Value
                         ?? throw new InvalidOperationException($"{linkName} has no cylinder radius.");
            return float.Parse(radius, CultureInfo.InvariantCulture);
        }

        private static float ReadWheelEffort(XDocument document)
        {
            var left = ReadJointLimit(document, "left_wheel_joint", "effort");
            var right = ReadJointLimit(document, "right_wheel_joint", "effort");
            if (!Mathf.Approximately(left, right))
            {
                throw new InvalidOperationException("Left and right wheel effort limits must match.");
            }

            return left;
        }

        private static float ReadJointLimit(XDocument document, string jointName, string attribute)
        {
            var joint = document.Root?.Elements("joint")
                .SingleOrDefault(item => (string)item.Attribute("name") == jointName)
                ?? throw new InvalidOperationException($"mCub URDF has no joint '{jointName}'.");
            return ParseFloat(
                joint.Element("limit") ?? throw new FormatException($"{jointName} has no limit."),
                attribute);
        }
        private static float ReadWheelTrack(XDocument document)
        {
            var left = JointY(document, "left_wheel_joint");
            var right = JointY(document, "right_wheel_joint");
            return Mathf.Abs(left - right);
        }

        private static float JointY(XDocument document, string jointName)
        {
            var joint = document.Root?.Elements("joint")
                .SingleOrDefault(item => (string)item.Attribute("name") == jointName)
                ?? throw new InvalidOperationException($"mCub URDF has no joint '{jointName}'.");
            var xyz = Required(joint.Element("origin"), "xyz")
                .Split((char[])null, StringSplitOptions.RemoveEmptyEntries);
            if (xyz.Length != 3)
            {
                throw new FormatException($"{jointName} origin xyz must contain three values.");
            }

            return float.Parse(xyz[1], CultureInfo.InvariantCulture);
        }

        private static Transform FindRequired(Transform root, string name)
        {
            return root.GetComponentsInChildren<Transform>(true)
                       .SingleOrDefault(item => item.name == name)
                   ?? throw new InvalidOperationException($"Imported mCub is missing '{name}'.");
        }

        private static void SetLayerRecursively(GameObject owner, int layer)
        {
            owner.layer = layer;
            foreach (Transform child in owner.transform)
            {
                SetLayerRecursively(child.gameObject, layer);
            }
        }

        private static string Required(XElement element, string attribute)
        {
            return element?.Attribute(attribute)?.Value
                   ?? throw new FormatException($"Missing required '{attribute}' attribute.");
        }

        private static int ParseInt(XElement element, string attribute)
        {
            return int.Parse(Required(element, attribute), CultureInfo.InvariantCulture);
        }

        private static float ParseFloat(XElement element, string attribute)
        {
            return float.Parse(Required(element, attribute), CultureInfo.InvariantCulture);
        }

        private static void SetFloat(SerializedObject owner, string name, float value)
        {
            RequiredProperty(owner, name).floatValue = value;
        }

        private static void SetInteger(SerializedObject owner, string name, int value)
        {
            RequiredProperty(owner, name).intValue = value;
        }

        private static void SetObject(SerializedObject owner, string name, UnityEngine.Object value)
        {
            RequiredProperty(owner, name).objectReferenceValue = value;
        }

        private static SerializedProperty RequiredProperty(SerializedObject owner, string name)
        {
            return owner.FindProperty(name)
                   ?? throw new MissingFieldException(owner.targetObject.GetType().FullName, name);
        }

        private readonly struct SensorConfig
        {
            public readonly string Link;
            public readonly int Samples;
            public readonly float Frequency;
            public readonly float MinRange;
            public readonly float MaxRange;

            public SensorConfig(string link, int samples, float frequency, float minRange, float maxRange)
            {
                Link = link;
                Samples = samples;
                Frequency = frequency;
                MinRange = minRange;
                MaxRange = maxRange;
            }
        }
    }
}
