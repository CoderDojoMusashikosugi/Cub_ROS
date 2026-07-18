using System;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Runtime.InteropServices;
using UnityEngine;

namespace CubSim
{
    public sealed class SimulationBootstrap : MonoBehaviour
    {
        private static readonly string[] WorldNames = { "Test World" };
        private static readonly string[] RobotNames = { "mCub" };

        private int selectedWorld;
        private int selectedRobot;
        private string spawnX = "0.0";
        private string spawnZ = "0.0";
        private string spawnYaw = "0.0";
        private string rosDomainId = "0";
        private string rosInterfaceIp = string.Empty;
        private string rosInterfaceDescription = string.Empty;
        private string rosTransportStatus = "not configured";
        private bool simulationRunning;
        private string validationMessage = string.Empty;
        private McubUrdfSpawner spawner;
        private FollowCamera followCamera;

        private void Awake()
        {
            Application.runInBackground = true;
#if CUB_ROS2
            NativeProcessEnvironment.Set("AMENT_PREFIX_PATH", NativePluginRoot());
#endif
        }

        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
        private static void EnsureBootstrap()
        {
            if (FindAnyObjectByType<SimulationBootstrap>() != null) return;
            new GameObject("SimulationBootstrap").AddComponent<SimulationBootstrap>();
        }

        private void Start()
        {
            Time.fixedDeltaTime = 0.005f;
            followCamera = CreateCamera();
            var arguments = Environment.GetCommandLineArgs();
            ApplyCommandLineOptions(arguments);
#if CUB_ROS2
            if (string.IsNullOrWhiteSpace(rosInterfaceIp) &&
                FastDdsNetworkConfigurator.TryDetectPreferredIpv4(out var detectedAddress, out var detectedDescription))
            {
                rosInterfaceIp = detectedAddress;
                rosInterfaceDescription = detectedDescription;
            }
#endif
            if (arguments.Contains("--local-smoke") || arguments.Contains("--auto-start"))
            {
                StartSimulation(arguments.Contains("--local-smoke"));
            }
        }

        private void ApplyCommandLineOptions(string[] arguments)
        {
            spawnX = OptionValue(arguments, "--spawn-x", spawnX);
            spawnZ = OptionValue(arguments, "--spawn-z", spawnZ);
            spawnYaw = OptionValue(arguments, "--spawn-yaw", spawnYaw);
            rosDomainId = OptionValue(arguments, "--ros-domain-id", rosDomainId);
            rosInterfaceIp = OptionValue(arguments, "--ros-interface-ip", rosInterfaceIp);
        }

        private static string OptionValue(string[] arguments, string option, string fallback)
        {
            var index = Array.IndexOf(arguments, option);
            return index >= 0 && index + 1 < arguments.Length ? arguments[index + 1] : fallback;
        }

        private void OnGUI()
        {
            if (simulationRunning)
            {
                DrawRunningStatus();
                return;
            }

            const float width = 430f;
            const float height = 455f;
            var area = new Rect((Screen.width - width) * 0.5f, (Screen.height - height) * 0.5f, width, height);
            GUILayout.BeginArea(area, GUI.skin.box);
            GUILayout.Space(12f);
            GUILayout.Label("CubSim Setup", HeaderStyle());
            GUILayout.Space(12f);

            GUILayout.Label("World");
            selectedWorld = GUILayout.SelectionGrid(selectedWorld, WorldNames, 1);
            GUILayout.Space(8f);
            GUILayout.Label("Robot");
            selectedRobot = GUILayout.SelectionGrid(selectedRobot, RobotNames, 1);
            GUILayout.Space(8f);

            GUILayout.Label("Initial pose (metres / degrees)");
            DrawTextField("X", ref spawnX);
            DrawTextField("Z", ref spawnZ);
            DrawTextField("Yaw", ref spawnYaw);
            DrawTextField("ROS domain ID", ref rosDomainId);
            DrawTextField("DDS interface IPv4", ref rosInterfaceIp);
            if (!string.IsNullOrEmpty(rosInterfaceDescription))
            {
                GUILayout.Label($"Auto-detected: {rosInterfaceDescription}", GUI.skin.label);
            }
            GUILayout.Space(8f);

            if (!string.IsNullOrEmpty(validationMessage))
            {
                GUILayout.Label(validationMessage, ErrorStyle());
            }

            GUILayout.FlexibleSpace();
            if (GUILayout.Button("Start Simulation", GUILayout.Height(42f)))
            {
                StartSimulation(false);
            }
            GUILayout.Space(10f);
            GUILayout.EndArea();
        }

        private void StartSimulation(bool localSmoke)
        {
            if (!TryParseSettings(out var position, out var yaw, out var domainId)) return;

#if CUB_ROS2
            if (!FastDdsNetworkConfigurator.IsValidIpv4(rosInterfaceIp))
            {
                validationMessage = "DDS interface must be a non-loopback IPv4 address.";
                return;
            }

            try
            {
                ConfigureRosTransport(domainId, rosInterfaceIp);
            }
            catch (Exception exception)
            {
                validationMessage = $"DDS setup failed: {exception.Message}";
                Debug.LogException(exception);
                return;
            }
#endif

            CreateWorld(selectedWorld);
            spawner = new GameObject("RobotSpawner").AddComponent<McubUrdfSpawner>();
            spawner.ConfigureSpawn(position, new Vector3(0f, yaw, 0f));
            followCamera.TargetProvider = () => spawner.SpawnedRobot == null ? null : spawner.SpawnedRobot.transform;
            if (localSmoke)
            {
                gameObject.AddComponent<LocalSimulationSmoke>().Spawner = spawner;
            }

            simulationRunning = true;
            validationMessage = string.Empty;
        }

        private bool TryParseSettings(out Vector3 position, out float yaw, out int domainId)
        {
            position = default;
            yaw = 0f;
            domainId = 0;
            if (!TryParseFloat(spawnX, out var x) || !TryParseFloat(spawnZ, out var z) || !TryParseFloat(spawnYaw, out yaw))
            {
                validationMessage = "Initial pose must contain valid numbers.";
                return false;
            }

            if (!int.TryParse(rosDomainId, NumberStyles.Integer, CultureInfo.InvariantCulture, out domainId) || domainId < 0 || domainId > 232)
            {
                validationMessage = "ROS domain ID must be between 0 and 232.";
                return false;
            }

            position = new Vector3(x, 0.004f, z);
            return true;
        }

        private static bool TryParseFloat(string value, out float parsed)
        {
            return float.TryParse(value, NumberStyles.Float, CultureInfo.InvariantCulture, out parsed);
        }

        private static void DrawTextField(string label, ref string value)
        {
            GUILayout.BeginHorizontal();
            GUILayout.Label(label, GUILayout.Width(130f));
            value = GUILayout.TextField(value);
            GUILayout.EndHorizontal();
        }

        private void DrawRunningStatus()
        {
            var ready = spawner != null && spawner.SpawnedRobot != null &&
                        spawner.SpawnedRobot.GetComponent<McubRos2Bridge>()?.IsRosReady == true;
            GUI.Box(new Rect(12f, 12f, 390f, 106f), string.Empty);
            GUI.Label(new Rect(24f, 22f, 270f, 22f), $"{WorldNames[selectedWorld]} / {RobotNames[selectedRobot]}");
            GUI.Label(new Rect(24f, 46f, 270f, 22f), ready ? "ROS 2: ready" : "ROS 2: waiting for Docker");
            GUI.Label(new Rect(24f, 68f, 355f, 22f), $"DDS: {rosTransportStatus}");
            GUI.Label(new Rect(24f, 90f, 355f, 22f), "Stop Play mode to change settings.");
        }

        private static GUIStyle HeaderStyle()
        {
            return new GUIStyle(GUI.skin.label) { fontSize = 24, alignment = TextAnchor.MiddleCenter };
        }

        private static GUIStyle ErrorStyle()
        {
            var style = new GUIStyle(GUI.skin.label) { wordWrap = true };
            style.normal.textColor = new Color(1f, 0.45f, 0.35f);
            return style;
        }

        private static void CreateWorld(int worldIndex)
        {
            if (worldIndex != 0) throw new ArgumentOutOfRangeException(nameof(worldIndex));
            if (GameObject.Find("TestWorld") != null) return;
            var world = new GameObject("TestWorld");
            CreateBox(world.transform, "Ground", new Vector3(0f, -0.05f, 0f), new Vector3(12f, 0.1f, 12f), new Color(0.25f, 0.27f, 0.3f));
            CreateBox(world.transform, "WallNorth", new Vector3(0f, 0.5f, 3f), new Vector3(6f, 1f, 0.1f), new Color(0.65f, 0.65f, 0.65f));
            CreateBox(world.transform, "WallWest", new Vector3(-3f, 0.5f, 0f), new Vector3(0.1f, 1f, 6f), new Color(0.65f, 0.65f, 0.65f));
            CreateBox(world.transform, "Obstacle", new Vector3(1f, 0.25f, 1.2f), new Vector3(0.5f, 0.5f, 0.5f), new Color(0.8f, 0.45f, 0.15f));

            if (FindAnyObjectByType<Light>() == null)
            {
                var light = new GameObject("Directional Light").AddComponent<Light>();
                light.type = LightType.Directional;
                light.intensity = 1.2f;
                light.transform.rotation = Quaternion.Euler(50f, -30f, 0f);
            }
        }

        private static FollowCamera CreateCamera()
        {
            var existing = Camera.main;
            var camera = existing == null ? new GameObject("Main Camera").AddComponent<Camera>() : existing;
            camera.tag = "MainCamera";
            camera.clearFlags = CameraClearFlags.SolidColor;
            camera.backgroundColor = new Color(0.08f, 0.09f, 0.12f);
            camera.transform.position = new Vector3(1.4f, 1.1f, -1.4f);
            camera.transform.rotation = Quaternion.Euler(25f, -40f, 0f);
            return camera.gameObject.AddComponent<FollowCamera>();
        }

        private static void CreateBox(Transform parent, string name, Vector3 position, Vector3 scale, Color color)
        {
            var box = GameObject.CreatePrimitive(PrimitiveType.Cube);
            box.name = name;
            box.transform.SetParent(parent);
            box.transform.position = position;
            box.transform.localScale = scale;
            box.GetComponent<Renderer>().material.color = color;
            var reflectivity = box.AddComponent<LidarReflectivity>();
            reflectivity.Value = (color.r + color.g + color.b) / 3f;
        }

        private static string NativePluginRoot()
        {
            return Application.isEditor
                ? Path.Combine(Application.dataPath, "Ros2ForUnity", "Plugins", "Windows", "x86_64")
                : Path.Combine(Application.dataPath, "Plugins", "x86_64");
        }

        private void ConfigureRosTransport(int domainId, string interfaceAddress)
        {
            NativeProcessEnvironment.Set("ROS_DOMAIN_ID", domainId.ToString(CultureInfo.InvariantCulture));
            NativeProcessEnvironment.Set("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp");
            NativeProcessEnvironment.Unset("ROS_DISCOVERY_SERVER");
            NativeProcessEnvironment.Unset("ROS_SUPER_CLIENT");
            NativeProcessEnvironment.Unset("ROS_LOCALHOST_ONLY");
            NativeProcessEnvironment.Set("ROS_AUTOMATIC_DISCOVERY_RANGE", "SYSTEM_DEFAULT");
            NativeProcessEnvironment.Set("RMW_FASTRTPS_USE_QOS_FROM_XML", "1");
            NativeProcessEnvironment.Unset("FASTDDS_BUILTIN_TRANSPORTS");
            var fastDdsProfile = FastDdsNetworkConfigurator.WriteUdpProfile(interfaceAddress);
            NativeProcessEnvironment.Set("FASTDDS_DEFAULT_PROFILES_FILE", fastDdsProfile);
            NativeProcessEnvironment.Set("FASTRTPS_DEFAULT_PROFILES_FILE", fastDdsProfile);
            rosTransportStatus = $"UDPv4 {interfaceAddress}, domain {domainId}";
            Debug.Log($"CubSim DDS configured: {rosTransportStatus}; profile={fastDdsProfile}");
        }

    }

    internal static class NativeProcessEnvironment
    {
#if UNITY_EDITOR_WIN || UNITY_STANDALONE_WIN
        [DllImport("ucrtbase.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        private static extern int _putenv_s(string name, string value);
#elif UNITY_EDITOR_LINUX || UNITY_STANDALONE_LINUX
        [DllImport("libc", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        private static extern int setenv(string name, string value, int overwrite);

        [DllImport("libc", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        private static extern int unsetenv(string name);
#endif

        public static void Set(string name, string value)
        {
            Environment.SetEnvironmentVariable(name, value, EnvironmentVariableTarget.Process);
#if UNITY_EDITOR_WIN || UNITY_STANDALONE_WIN
            var result = _putenv_s(name, value);
            if (result != 0) throw new InvalidOperationException($"_putenv_s failed for {name}: {result}");
#elif UNITY_EDITOR_LINUX || UNITY_STANDALONE_LINUX
            var result = setenv(name, value, 1);
            if (result != 0) throw new InvalidOperationException($"setenv failed for {name}: {result}");
#endif
        }

        public static void Unset(string name)
        {
            Environment.SetEnvironmentVariable(name, null, EnvironmentVariableTarget.Process);
#if UNITY_EDITOR_WIN || UNITY_STANDALONE_WIN
            var result = _putenv_s(name, string.Empty);
            if (result != 0) throw new InvalidOperationException($"_putenv_s failed for {name}: {result}");
#elif UNITY_EDITOR_LINUX || UNITY_STANDALONE_LINUX
            var result = unsetenv(name);
            if (result != 0) throw new InvalidOperationException($"unsetenv failed for {name}: {result}");
#endif
        }
    }

    public sealed class LocalSimulationSmoke : MonoBehaviour
    {
        public McubUrdfSpawner Spawner;
        private SlamtecC1Lidar lidar;
        private int scanCount;
        private float startedAt;
        private double initialX;
        private bool initialized;

        private void Start()
        {
            startedAt = Time.realtimeSinceStartup;
        }

        private void Update()
        {
            var robot = Spawner == null ? null : Spawner.SpawnedRobot;
            if (!initialized && robot != null)
            {
                initialized = true;
                initialX = robot.Odometry.X;
                lidar = robot.GetComponentInChildren<SlamtecC1Lidar>();
                lidar.ScanReady += OnScan;
            }

            if (!initialized) return;
            robot.SetVelocityCommand(0.1f, 0f);
            var elapsed = Time.realtimeSinceStartup - startedAt;
            var travel = Math.Abs(robot.Odometry.X - initialX);
            if (scanCount >= 10 && travel >= 0.02)
            {
                Debug.Log($"LOCAL_SMOKE_PASSED scans={scanCount} travel={travel:F3}m");
                Application.Quit(0);
            }
            else if (elapsed > 10f)
            {
                Debug.LogError($"LOCAL_SMOKE_FAILED scans={scanCount} travel={travel:F3}m");
                Application.Quit(1);
            }
        }

        private void OnScan(C1Scan scan)
        {
            if (scan.Ranges.Length == SlamtecC1Lidar.SamplesPerScan) scanCount++;
        }

        private void OnDestroy()
        {
            if (lidar != null) lidar.ScanReady -= OnScan;
        }
    }

    public sealed class FollowCamera : MonoBehaviour
    {
        public Func<Transform> TargetProvider;
        private readonly Vector3 offset = new Vector3(1.4f, 1.1f, -1.4f);

        private void LateUpdate()
        {
            var target = TargetProvider?.Invoke();
            if (target == null) return;
            transform.position = Vector3.Lerp(transform.position, target.position + offset, 4f * Time.deltaTime);
            transform.LookAt(target.position + Vector3.up * 0.08f);
        }
    }
}
