using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using UnityEngine;

namespace CubSim
{
    public sealed class McubUrdfSpawner : MonoBehaviour
    {
        [SerializeField] private string urdfRelativePath = "Robots/mcub.urdf";
        [SerializeField] private Vector3 spawnPosition = new Vector3(0f, 0.004f, 0f);
        [SerializeField] private Vector3 spawnEulerAngles;

        public McubRobot SpawnedRobot { get; private set; }

        private void Start()
        {
            Spawn();
        }

        public void ConfigureSpawn(Vector3 position, Vector3 eulerAngles)
        {
            if (SpawnedRobot != null) throw new InvalidOperationException("Cannot change the spawn pose after spawning.");
            spawnPosition = position;
            spawnEulerAngles = eulerAngles;
        }

        public McubRobot Spawn()
        {
            if (SpawnedRobot != null)
            {
                Destroy(SpawnedRobot.transform.root.gameObject);
            }

            var path = Path.Combine(Application.streamingAssetsPath, urdfRelativePath);
            if (!File.Exists(path))
            {
                throw new FileNotFoundException(
                    "Generated mCub URDF is missing. Run unity/scripts/update_mcub_model.ps1 (or .sh).",
                    path);
            }

            return SpawnFromUrdf(File.ReadAllText(path));
        }

        public McubRobot SpawnFromUrdf(string urdf)
        {
            var model = UrdfModel.Parse(urdf);
            ValidateMcubModel(model);

            var container = new GameObject(model.Name);
            container.transform.SetPositionAndRotation(spawnPosition, Quaternion.Euler(spawnEulerAngles));
            var links = BuildLinkHierarchy(model, container.transform);
            foreach (var link in model.Links.Values)
            {
                AddVisuals(links[link.Name], link);
            }

            SetLayerRecursively(container, Physics.IgnoreRaycastLayer);

            var baseLink = links[model.RootLinkName].gameObject;
            var body = baseLink.AddComponent<Rigidbody>();
            var baseModel = model.Links[model.RootLinkName];
            body.mass = Mathf.Max(0.01f, baseModel.Mass);
            body.centerOfMass = RosUnityCoordinates.Position(baseModel.InertialOrigin.PositionRos);
            body.interpolation = RigidbodyInterpolation.Interpolate;
            body.collisionDetectionMode = CollisionDetectionMode.ContinuousDynamic;
            body.maxAngularVelocity = 30f;
            AddBaseCollision(baseLink, baseModel);

            var leftJoint = model.Joint("left_wheel_joint");
            var rightJoint = model.Joint("right_wheel_joint");
            var leftGeometry = model.Links[leftJoint.Child].Visuals.Single(item => item.Type == UrdfGeometryType.Cylinder);
            var rightGeometry = model.Links[rightJoint.Child].Visuals.Single(item => item.Type == UrdfGeometryType.Cylinder);
            var leftWheel = AddWheelCollider(baseLink.transform, leftJoint, leftGeometry);
            var rightWheel = AddWheelCollider(baseLink.transform, rightJoint, rightGeometry);
            AddCasterCollision(links, model);

            var robot = baseLink.AddComponent<McubRobot>();
            var track = Mathf.Abs(leftJoint.Origin.PositionRos.y - rightJoint.Origin.PositionRos.y);
            robot.Configure(
                body,
                leftWheel,
                rightWheel,
                FindVisual(links[leftJoint.Child]),
                FindVisual(links[rightJoint.Child]),
                leftGeometry.Radius,
                track);

            var lidar = links["SLC1_link"].gameObject.AddComponent<SlamtecC1Lidar>();
            var bridge = baseLink.AddComponent<McubRos2Bridge>();
            bridge.Initialize(robot, lidar);
            SpawnedRobot = robot;
            return robot;
        }

        public static void ValidateMcubModel(UrdfModel model)
        {
            var requiredLinks = new[]
            {
                "base_link", "left_wheel_link", "right_wheel_link", "caster_wheel_link", "SLC1_link", "SLC1_base"
            };
            foreach (var link in requiredLinks)
            {
                if (!model.Links.ContainsKey(link)) throw new FormatException($"mCub URDF is missing link '{link}'.");
            }

            var left = model.Joint("left_wheel_joint");
            var right = model.Joint("right_wheel_joint");
            var radius = model.Links[left.Child].Visuals.Single(item => item.Type == UrdfGeometryType.Cylinder).Radius;
            var track = Mathf.Abs(left.Origin.PositionRos.y - right.Origin.PositionRos.y);
            if (Mathf.Abs(radius - 0.02f) > 1e-5f) throw new FormatException("mCub drive wheel radius must currently be 0.020 m.");
            if (track <= 0f) throw new FormatException("mCub wheel track must be positive.");

            var lidarHead = model.Links["SLC1_link"].Visuals.Single(item => item.Type == UrdfGeometryType.Cylinder);
            if (lidarHead.Origin.PositionRos.sqrMagnitude > 1e-10f)
            {
                throw new FormatException("SLC1 cylinder center must remain at the SLC1_link scan plane.");
            }
        }

        private static Dictionary<string, Transform> BuildLinkHierarchy(UrdfModel model, Transform container)
        {
            var links = model.Links.Keys.ToDictionary(name => name, name => new GameObject(name).transform);
            var root = links[model.RootLinkName];
            root.SetParent(container, false);
            var remaining = new HashSet<UrdfJoint>(model.Joints);
            var built = new HashSet<string> { model.RootLinkName };
            while (remaining.Count > 0)
            {
                var progress = false;
                foreach (var joint in remaining.ToArray())
                {
                    if (!built.Contains(joint.Parent)) continue;
                    var child = links[joint.Child];
                    child.SetParent(links[joint.Parent], false);
                    child.localPosition = RosUnityCoordinates.Position(joint.Origin.PositionRos);
                    child.localRotation = RosUnityCoordinates.Rotation(joint.Origin.RpyRos);
                    built.Add(joint.Child);
                    remaining.Remove(joint);
                    progress = true;
                }

                if (!progress) throw new FormatException("URDF joint graph is disconnected or cyclic.");
            }

            return links;
        }

        private static void AddVisuals(Transform linkTransform, UrdfLink link)
        {
            for (var index = 0; index < link.Visuals.Count; index++)
            {
                var geometry = link.Visuals[index];
                var primitive = GameObject.CreatePrimitive(
                    geometry.Type == UrdfGeometryType.Box ? PrimitiveType.Cube : PrimitiveType.Cylinder);
                primitive.name = $"visual_{index}_{geometry.Type.ToString().ToLowerInvariant()}";
                primitive.transform.SetParent(linkTransform, false);
                primitive.transform.localPosition = RosUnityCoordinates.Position(geometry.Origin.PositionRos);
                primitive.transform.localRotation = RosUnityCoordinates.Rotation(geometry.Origin.RpyRos);
                primitive.transform.localScale = geometry.Type == UrdfGeometryType.Box
                    ? RosUnityCoordinates.Scale(geometry.SizeRos)
                    : new Vector3(geometry.Radius * 2f, geometry.Length * 0.5f, geometry.Radius * 2f);
                var collider = primitive.GetComponent<Collider>();
                if (collider != null) Destroy(collider);
                var renderer = primitive.GetComponent<Renderer>();
                renderer.material.color = geometry.Color;
            }
        }

        private static void AddBaseCollision(GameObject baseLink, UrdfLink link)
        {
            var collision = link.Collisions.FirstOrDefault(item => item.Type == UrdfGeometryType.Box)
                            ?? throw new FormatException("base_link needs a box collision.");
            var collider = baseLink.AddComponent<BoxCollider>();
            collider.center = RosUnityCoordinates.Position(collision.Origin.PositionRos);
            collider.size = RosUnityCoordinates.Scale(collision.SizeRos);
        }

        private static WheelCollider AddWheelCollider(Transform baseLink, UrdfJoint joint, UrdfGeometry geometry)
        {
            var wheelObject = new GameObject(joint.Name + "_physics");
            wheelObject.layer = Physics.IgnoreRaycastLayer;
            wheelObject.transform.SetParent(baseLink, false);
            wheelObject.transform.localPosition = RosUnityCoordinates.Position(joint.Origin.PositionRos);
            var wheel = wheelObject.AddComponent<WheelCollider>();
            wheel.radius = geometry.Radius;
            wheel.mass = 0.05f;
            wheel.suspensionDistance = 0.003f;
            wheel.forceAppPointDistance = 0f;
            var spring = wheel.suspensionSpring;
            spring.spring = 3500f;
            spring.damper = 160f;
            spring.targetPosition = 0.5f;
            wheel.suspensionSpring = spring;
            var forward = wheel.forwardFriction;
            forward.stiffness = 2f;
            wheel.forwardFriction = forward;
            var sideways = wheel.sidewaysFriction;
            sideways.stiffness = 2.5f;
            wheel.sidewaysFriction = sideways;
            return wheel;
        }

        private static void AddCasterCollision(IReadOnlyDictionary<string, Transform> links, UrdfModel model)
        {
            var caster = model.Links["caster_wheel_link"].Visuals.Single(item => item.Type == UrdfGeometryType.Cylinder);
            var sphere = links["caster_wheel_link"].gameObject.AddComponent<SphereCollider>();
            sphere.radius = caster.Radius;
        }

        private static Transform FindVisual(Transform link)
        {
            return link.childCount == 0 ? null : link.GetChild(0);
        }

        private static void SetLayerRecursively(GameObject owner, int layer)
        {
            owner.layer = layer;
            foreach (Transform child in owner.transform) SetLayerRecursively(child.gameObject, layer);
        }
    }
}
