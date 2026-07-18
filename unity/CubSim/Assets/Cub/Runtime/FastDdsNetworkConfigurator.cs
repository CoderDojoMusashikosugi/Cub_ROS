using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Net;
using System.Net.NetworkInformation;
using System.Net.Sockets;
using System.Text;
using UnityEngine;

namespace CubSim
{
    public static class FastDdsNetworkConfigurator
    {
        private const string ProfileTemplateName = "fastdds-windows.xml";
        private const string InterfaceAddressToken = "__INTERFACE_ADDRESS__";

        public static bool TryDetectPreferredIpv4(out string address, out string description)
        {
            address = string.Empty;
            description = string.Empty;

            try
            {
                var candidates = new List<InterfaceCandidate>();
                var preferWslAdapter = !IsWslMirroredMode();
                foreach (var networkInterface in NetworkInterface.GetAllNetworkInterfaces())
                {
                    if (networkInterface.OperationalStatus != OperationalStatus.Up ||
                        networkInterface.NetworkInterfaceType == NetworkInterfaceType.Loopback ||
                        networkInterface.NetworkInterfaceType == NetworkInterfaceType.Tunnel)
                    {
                        continue;
                    }

                    var properties = networkInterface.GetIPProperties();
                    foreach (var unicast in properties.UnicastAddresses)
                    {
                        var candidateAddress = unicast.Address;
                        if (candidateAddress.AddressFamily != AddressFamily.InterNetwork ||
                            IPAddress.IsLoopback(candidateAddress) ||
                            candidateAddress.GetAddressBytes()[0] == 169)
                        {
                            continue;
                        }

                        candidates.Add(new InterfaceCandidate(
                            candidateAddress.ToString(),
                            networkInterface.Name,
                            networkInterface.Description,
                            Score(networkInterface, properties, candidateAddress, preferWslAdapter)));
                    }
                }

                var selected = candidates
                    .OrderByDescending(candidate => candidate.Score)
                    .ThenBy(candidate => candidate.Name, StringComparer.OrdinalIgnoreCase)
                    .FirstOrDefault();
                if (selected == null) return false;

                address = selected.Address;
                description = selected.Name == selected.Description
                    ? selected.Name
                    : $"{selected.Name} ({selected.Description})";
                return true;
            }
            catch (NetworkInformationException exception)
            {
                Debug.LogWarning($"Could not enumerate network interfaces for Fast DDS: {exception.Message}");
                return false;
            }
        }

        public static bool IsValidIpv4(string value)
        {
            return IPAddress.TryParse(value, out var address) &&
                   address.AddressFamily == AddressFamily.InterNetwork &&
                   !IPAddress.IsLoopback(address) &&
                   address.GetAddressBytes()[0] != 169;
        }

        public static string WriteUdpProfile(string interfaceAddress)
        {
            if (!IsValidIpv4(interfaceAddress))
                throw new ArgumentException("A non-loopback IPv4 address is required.", nameof(interfaceAddress));

            var templatePath = Path.Combine(Application.streamingAssetsPath, ProfileTemplateName);
            if (!File.Exists(templatePath))
                throw new FileNotFoundException("Fast DDS profile template is missing.", templatePath);

            var template = File.ReadAllText(templatePath);
            if (!template.Contains(InterfaceAddressToken))
                throw new InvalidDataException($"Fast DDS profile template does not contain {InterfaceAddressToken}.");

            var outputDirectory = Path.Combine(Application.persistentDataPath, "FastDDS");
            Directory.CreateDirectory(outputDirectory);
            var outputPath = Path.Combine(outputDirectory, "cubsim-udp.xml");
            File.WriteAllText(
                outputPath,
                template.Replace(InterfaceAddressToken, interfaceAddress),
                new UTF8Encoding(false));
            return outputPath;
        }

        private static int Score(
            NetworkInterface networkInterface,
            IPInterfaceProperties properties,
            IPAddress address,
            bool preferWslAdapter)
        {
            var label = $"{networkInterface.Name} {networkInterface.Description}";
            var score = 0;
            if (label.IndexOf("WSL", StringComparison.OrdinalIgnoreCase) >= 0)
                score += preferWslAdapter ? 1000 : -1000;
            if (label.IndexOf("Hyper-V", StringComparison.OrdinalIgnoreCase) >= 0) score += 300;
            if (properties.GatewayAddresses.Any(gateway =>
                    gateway.Address.AddressFamily == AddressFamily.InterNetwork &&
                    !gateway.Address.Equals(IPAddress.Any))) score += 100;
            if (networkInterface.NetworkInterfaceType == NetworkInterfaceType.Ethernet ||
                networkInterface.NetworkInterfaceType == NetworkInterfaceType.Wireless80211) score += 20;
            if (IsPrivate(address)) score += 10;
            return score;
        }

        private static bool IsWslMirroredMode()
        {
            try
            {
                var userProfile = Environment.GetFolderPath(Environment.SpecialFolder.UserProfile);
                var configurationPath = Path.Combine(userProfile, ".wslconfig");
                if (!File.Exists(configurationPath)) return false;
                return File.ReadLines(configurationPath)
                    .Select(line => line.Replace(" ", string.Empty).Replace("\t", string.Empty))
                    .Any(line => line.Equals("networkingMode=mirrored", StringComparison.OrdinalIgnoreCase));
            }
            catch (IOException)
            {
                return false;
            }
            catch (UnauthorizedAccessException)
            {
                return false;
            }
        }

        private static bool IsPrivate(IPAddress address)
        {
            var bytes = address.GetAddressBytes();
            return bytes[0] == 10 ||
                   (bytes[0] == 172 && bytes[1] >= 16 && bytes[1] <= 31) ||
                   (bytes[0] == 192 && bytes[1] == 168);
        }

        private sealed class InterfaceCandidate
        {
            public InterfaceCandidate(string address, string name, string description, int score)
            {
                Address = address;
                Name = name;
                Description = description;
                Score = score;
            }

            public string Address { get; }
            public string Name { get; }
            public string Description { get; }
            public int Score { get; }
        }
    }
}
