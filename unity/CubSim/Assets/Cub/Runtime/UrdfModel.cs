using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using System.Xml.Linq;
using UnityEngine;

namespace CubSim
{
    public enum UrdfGeometryType
    {
        Box,
        Cylinder
    }

    public readonly struct UrdfPose
    {
        public readonly Vector3 PositionRos;
        public readonly Vector3 RpyRos;

        public UrdfPose(Vector3 positionRos, Vector3 rpyRos)
        {
            PositionRos = positionRos;
            RpyRos = rpyRos;
        }
    }

    public sealed class UrdfGeometry
    {
        public UrdfGeometryType Type;
        public UrdfPose Origin;
        public Vector3 SizeRos;
        public float Radius;
        public float Length;
        public Color Color = Color.gray;
    }

    public sealed class UrdfLink
    {
        public string Name;
        public float Mass = 1f;
        public UrdfPose InertialOrigin;
        public readonly List<UrdfGeometry> Visuals = new List<UrdfGeometry>();
        public readonly List<UrdfGeometry> Collisions = new List<UrdfGeometry>();
    }

    public sealed class UrdfJoint
    {
        public string Name;
        public string Type;
        public string Parent;
        public string Child;
        public UrdfPose Origin;
        public Vector3 AxisRos = Vector3.right;
    }

    public sealed class UrdfModel
    {
        public string Name;
        public string RootLinkName;
        public readonly Dictionary<string, UrdfLink> Links = new Dictionary<string, UrdfLink>();
        public readonly List<UrdfJoint> Joints = new List<UrdfJoint>();

        public static UrdfModel Parse(string xml)
        {
            if (string.IsNullOrWhiteSpace(xml))
            {
                throw new ArgumentException("URDF is empty.", nameof(xml));
            }

            if (xml.Contains("${") || xml.Contains("$(arg"))
            {
                throw new FormatException("The Unity input must be expanded URDF, not xacro.");
            }

            var document = XDocument.Parse(xml, LoadOptions.SetLineInfo);
            var robot = document.Root;
            if (robot == null || robot.Name.LocalName != "robot")
            {
                throw new FormatException("URDF root element must be <robot>.");
            }

            var model = new UrdfModel { Name = RequiredAttribute(robot, "name") };
            var materials = ParseMaterials(robot);

            foreach (var linkElement in robot.Elements("link"))
            {
                var link = new UrdfLink { Name = RequiredAttribute(linkElement, "name") };
                var inertial = linkElement.Element("inertial");
                if (inertial != null)
                {
                    link.InertialOrigin = ParseOrigin(inertial.Element("origin"));
                    link.Mass = ParseFloat(inertial.Element("mass")?.Attribute("value")?.Value, 1f);
                }

                foreach (var visual in linkElement.Elements("visual"))
                {
                    link.Visuals.Add(ParseGeometry(visual, materials));
                }

                foreach (var collision in linkElement.Elements("collision"))
                {
                    link.Collisions.Add(ParseGeometry(collision, materials));
                }

                model.Links.Add(link.Name, link);
            }

            foreach (var jointElement in robot.Elements("joint"))
            {
                model.Joints.Add(new UrdfJoint
                {
                    Name = RequiredAttribute(jointElement, "name"),
                    Type = RequiredAttribute(jointElement, "type"),
                    Parent = RequiredAttribute(jointElement.Element("parent"), "link"),
                    Child = RequiredAttribute(jointElement.Element("child"), "link"),
                    Origin = ParseOrigin(jointElement.Element("origin")),
                    AxisRos = ParseVector(jointElement.Element("axis")?.Attribute("xyz")?.Value, Vector3.right)
                });
            }

            var childLinks = new HashSet<string>(model.Joints.Select(joint => joint.Child));
            model.RootLinkName = model.Links.Keys.SingleOrDefault(name => !childLinks.Contains(name));
            if (model.RootLinkName == null)
            {
                throw new FormatException("URDF must have exactly one root link.");
            }

            foreach (var joint in model.Joints)
            {
                if (!model.Links.ContainsKey(joint.Parent) || !model.Links.ContainsKey(joint.Child))
                {
                    throw new FormatException($"Joint '{joint.Name}' references an unknown link.");
                }
            }

            return model;
        }

        public UrdfJoint Joint(string name)
        {
            return Joints.Single(joint => joint.Name == name);
        }

        private static Dictionary<string, Color> ParseMaterials(XElement robot)
        {
            var result = new Dictionary<string, Color>();
            foreach (var element in robot.Elements("material"))
            {
                var name = element.Attribute("name")?.Value;
                var rgba = element.Element("color")?.Attribute("rgba")?.Value;
                if (!string.IsNullOrEmpty(name) && !string.IsNullOrEmpty(rgba))
                {
                    result[name] = ParseColor(rgba);
                }
            }

            return result;
        }

        private static UrdfGeometry ParseGeometry(XElement owner, IReadOnlyDictionary<string, Color> materials)
        {
            var geometryElement = owner.Element("geometry") ?? throw new FormatException("visual/collision has no geometry.");
            var geometry = new UrdfGeometry { Origin = ParseOrigin(owner.Element("origin")) };
            var box = geometryElement.Element("box");
            var cylinder = geometryElement.Element("cylinder");
            if (box != null)
            {
                geometry.Type = UrdfGeometryType.Box;
                geometry.SizeRos = ParseVector(RequiredAttribute(box, "size"), Vector3.one);
            }
            else if (cylinder != null)
            {
                geometry.Type = UrdfGeometryType.Cylinder;
                geometry.Radius = ParseFloat(RequiredAttribute(cylinder, "radius"));
                geometry.Length = ParseFloat(RequiredAttribute(cylinder, "length"));
            }
            else
            {
                throw new NotSupportedException("The mCub test importer currently supports box and cylinder geometry.");
            }

            var material = owner.Element("material");
            var inlineColor = material?.Element("color")?.Attribute("rgba")?.Value;
            var materialName = material?.Attribute("name")?.Value;
            if (!string.IsNullOrEmpty(inlineColor))
            {
                geometry.Color = ParseColor(inlineColor);
            }
            else if (!string.IsNullOrEmpty(materialName) && materials.TryGetValue(materialName, out var color))
            {
                geometry.Color = color;
            }

            return geometry;
        }

        private static UrdfPose ParseOrigin(XElement origin)
        {
            if (origin == null)
            {
                return new UrdfPose(Vector3.zero, Vector3.zero);
            }

            return new UrdfPose(
                ParseVector(origin.Attribute("xyz")?.Value, Vector3.zero),
                ParseVector(origin.Attribute("rpy")?.Value, Vector3.zero));
        }

        private static Vector3 ParseVector(string value, Vector3 fallback)
        {
            if (string.IsNullOrWhiteSpace(value))
            {
                return fallback;
            }

            var values = value.Split((char[])null, StringSplitOptions.RemoveEmptyEntries);
            if (values.Length != 3)
            {
                throw new FormatException($"Expected three numbers, got '{value}'.");
            }

            return new Vector3(ParseFloat(values[0]), ParseFloat(values[1]), ParseFloat(values[2]));
        }

        private static Color ParseColor(string value)
        {
            var values = value.Split((char[])null, StringSplitOptions.RemoveEmptyEntries);
            if (values.Length != 4)
            {
                throw new FormatException($"Expected RGBA, got '{value}'.");
            }

            return new Color(ParseFloat(values[0]), ParseFloat(values[1]), ParseFloat(values[2]), ParseFloat(values[3]));
        }

        private static float ParseFloat(string value, float fallback = 0f)
        {
            return string.IsNullOrWhiteSpace(value)
                ? fallback
                : float.Parse(value, NumberStyles.Float, CultureInfo.InvariantCulture);
        }

        private static string RequiredAttribute(XElement element, string name)
        {
            return element?.Attribute(name)?.Value
                   ?? throw new FormatException($"Required attribute '{name}' is missing.");
        }
    }

    public static class RosUnityCoordinates
    {
        public static Vector3 Position(Vector3 ros)
        {
            return new Vector3(-ros.y, ros.z, ros.x);
        }

        public static Vector3 Scale(Vector3 ros)
        {
            return new Vector3(ros.y, ros.z, ros.x);
        }

        public static Quaternion Rotation(Vector3 rpy)
        {
            var roll = rpy.x * 0.5f;
            var pitch = rpy.y * 0.5f;
            var yaw = rpy.z * 0.5f;
            var cr = Mathf.Cos(roll);
            var sr = Mathf.Sin(roll);
            var cp = Mathf.Cos(pitch);
            var sp = Mathf.Sin(pitch);
            var cy = Mathf.Cos(yaw);
            var sy = Mathf.Sin(yaw);
            var ros = new Quaternion(
                sr * cp * cy - cr * sp * sy,
                cr * sp * cy + sr * cp * sy,
                cr * cp * sy - sr * sp * cy,
                cr * cp * cy + sr * sp * sy);
            return new Quaternion(ros.y, -ros.z, -ros.x, ros.w);
        }
    }
}
