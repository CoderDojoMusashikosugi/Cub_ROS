using System;
using System.Linq;
using UnityEngine;

namespace CubSim
{
    public sealed class McubPrefabSpawner : MonoBehaviour
    {
        [SerializeField] private string prefabResourcePath = "mcub";
        [SerializeField] private Vector3 spawnPosition = new Vector3(0f, 0.004f, 0f);
        [SerializeField] private Vector3 spawnEulerAngles;

        private GameObject spawnedInstance;

        public McubRobot SpawnedRobot { get; private set; }

        private void Start()
        {
            Spawn();
        }

        public void ConfigureSpawn(Vector3 position, Vector3 eulerAngles)
        {
            if (SpawnedRobot != null)
            {
                throw new InvalidOperationException("Cannot change the spawn pose after spawning.");
            }

            spawnPosition = position;
            spawnEulerAngles = eulerAngles;
        }

        public McubRobot Spawn()
        {
            if (spawnedInstance != null)
            {
                Destroy(spawnedInstance);
            }

            var prefab = Resources.Load<GameObject>(prefabResourcePath);
            if (prefab == null)
            {
                throw new InvalidOperationException(
                    "Generated mCub prefab is missing. Run unity/scripts/update_mcub_model.ps1 (or .sh).");
            }

            spawnedInstance = Instantiate(
                prefab,
                spawnPosition,
                Quaternion.Euler(spawnEulerAngles));
            spawnedInstance.name = "mcub";

            var robot = spawnedInstance.GetComponentInChildren<McubRobot>(true);
            if (robot == null)
            {
                throw new InvalidOperationException("Generated mCub prefab has no McubRobot component.");
            }

            var lidar = spawnedInstance
                .GetComponentsInChildren<MonoBehaviour>(true)
                .OfType<IC1ScanSource>()
                .SingleOrDefault();
            if (lidar == null)
            {
                throw new InvalidOperationException("Generated mCub prefab has no C1 scan source.");
            }

            var bridge = robot.GetComponent<McubRos2Bridge>();
            if (bridge == null)
            {
                bridge = robot.gameObject.AddComponent<McubRos2Bridge>();
            }

            bridge.Initialize(robot, lidar);
            SpawnedRobot = robot;
            return robot;
        }
    }
}
