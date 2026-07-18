using System;
using System.Runtime.InteropServices;
using System.Text;
using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.InputSystem.Controls;

namespace CubSim
{
    internal sealed class GamepadJoySample
    {
        public string ControllerType;
        public string Description;
        public int DeviceId;
        public float[] Axes;
        public int[] Buttons;
    }

    internal static class UnityGamepadJoySource
    {
        public static string DescribeDevices()
        {
            var description = new StringBuilder();
            description.Append($"Gamepad.all={Gamepad.all.Count}; InputSystem.devices={InputSystem.devices.Count}");
            foreach (var device in InputSystem.devices)
            {
                description.Append("; ");
                description.Append(device.displayName);
                description.Append(" [layout=");
                description.Append(device.layout);
                description.Append(", interface=");
                description.Append(device.description.interfaceName);
                description.Append(", product=");
                description.Append(device.description.product);
                description.Append(']');
            }
            return description.ToString();
        }

        public static bool TryRead(out GamepadJoySample sample)
        {
            sample = null;

#if UNITY_STANDALONE_WIN || UNITY_EDITOR_WIN
            // Unity's Editor focus handling can suspend XInput events even when the
            // simulation itself keeps running. Poll XInput in-process so an Xbox
            // controller remains usable while RViz or a terminal has focus.
            if (WindowsXInput.TryRead(out sample)) return true;
#endif

            var gamepad = SelectGamepad();
            if (gamepad == null) return false;

            var controllerType = Classify(gamepad);
            if (controllerType == null) return false;

            sample = controllerType == "xbox"
                ? ReadXbox(gamepad)
                : ReadDualSense(gamepad);
            return true;
        }

        private static Gamepad SelectGamepad()
        {
            if (Gamepad.current != null && Classify(Gamepad.current) != null)
                return Gamepad.current;

            foreach (var gamepad in Gamepad.all)
            {
                if (Classify(gamepad) != null) return gamepad;
            }
            return null;
        }

        private static string Classify(Gamepad gamepad)
        {
            var identity = string.Join(" ", new[]
            {
                gamepad.layout,
                gamepad.displayName,
                gamepad.description.interfaceName,
                gamepad.description.manufacturer,
                gamepad.description.product
            }).ToLowerInvariant();

            if (identity.Contains("xbox") || identity.Contains("xinput")) return "xbox";
            if (identity.Contains("dualsense") || identity.Contains("dual sense")) return "dualsense";
            return null;
        }

        private static GamepadJoySample ReadXbox(Gamepad gamepad)
        {
            var left = gamepad.leftStick.ReadValue();
            var right = gamepad.rightStick.ReadValue();
            var dpad = gamepad.dpad.ReadValue();
            var axes = new float[8];
            axes[0] = -left.x;
            axes[1] = left.y;
            axes[2] = right.y;
            axes[3] = -right.x;
            axes[4] = TriggerAxis(gamepad.rightTrigger.ReadValue());
            axes[5] = TriggerAxis(gamepad.leftTrigger.ReadValue());
            axes[6] = -dpad.x;
            axes[7] = dpad.y;

            // Match the Linux js event numbering consumed by JoyToXBox1914.
            var buttons = new int[15];
            buttons[0] = Pressed(gamepad.buttonSouth);
            buttons[1] = Pressed(gamepad.buttonEast);
            buttons[3] = Pressed(gamepad.buttonWest);
            buttons[4] = Pressed(gamepad.buttonNorth);
            buttons[6] = Pressed(gamepad.leftShoulder);
            buttons[7] = Pressed(gamepad.rightShoulder);
            buttons[10] = Pressed(gamepad.selectButton);
            buttons[11] = Pressed(gamepad.startButton);
            buttons[12] = ReadNamedButton(gamepad, "systemButton", "homeButton", "guideButton");
            buttons[13] = Pressed(gamepad.leftStickButton);
            buttons[14] = Pressed(gamepad.rightStickButton);
            return CreateSample(gamepad, "xbox", axes, buttons);
        }

        private static GamepadJoySample ReadDualSense(Gamepad gamepad)
        {
            var left = gamepad.leftStick.ReadValue();
            var right = gamepad.rightStick.ReadValue();
            var dpad = gamepad.dpad.ReadValue();
            var axes = new float[8];
            axes[0] = -left.x;
            axes[1] = left.y;
            axes[2] = TriggerAxis(gamepad.leftTrigger.ReadValue());
            axes[3] = right.y;
            axes[4] = -right.x;
            axes[5] = TriggerAxis(gamepad.rightTrigger.ReadValue());
            axes[6] = -dpad.x;
            axes[7] = dpad.y;

            // Match the Linux js event numbering consumed by JoyToDualSense.
            var buttons = new int[14];
            buttons[0] = Pressed(gamepad.buttonSouth);
            buttons[1] = Pressed(gamepad.buttonEast);
            buttons[2] = Pressed(gamepad.buttonNorth);
            buttons[3] = Pressed(gamepad.buttonWest);
            buttons[4] = Pressed(gamepad.leftShoulder);
            buttons[5] = Pressed(gamepad.rightShoulder);
            buttons[6] = gamepad.leftTrigger.isPressed ? 1 : 0;
            buttons[7] = gamepad.rightTrigger.isPressed ? 1 : 0;
            buttons[8] = Pressed(gamepad.selectButton);
            buttons[9] = Pressed(gamepad.startButton);
            buttons[10] = Pressed(gamepad.leftStickButton);
            buttons[11] = Pressed(gamepad.rightStickButton);
            buttons[12] = ReadNamedButton(gamepad, "systemButton", "homeButton");
            buttons[13] = ReadNamedButton(gamepad, "touchpadButton");
            return CreateSample(gamepad, "dualsense", axes, buttons);
        }

        private static GamepadJoySample CreateSample(
            Gamepad gamepad,
            string controllerType,
            float[] axes,
            int[] buttons)
        {
            var product = string.IsNullOrWhiteSpace(gamepad.description.product)
                ? gamepad.displayName
                : gamepad.description.product;
            return new GamepadJoySample
            {
                ControllerType = controllerType,
                Description = product,
                DeviceId = gamepad.deviceId,
                Axes = axes,
                Buttons = buttons
            };
        }

        private static float TriggerAxis(float value)
        {
            return Mathf.Clamp(1f - 2f * value, -1f, 1f);
        }

        private static int Pressed(ButtonControl button)
        {
            return button != null && button.isPressed ? 1 : 0;
        }

        private static int ReadNamedButton(Gamepad gamepad, params string[] names)
        {
            foreach (var name in names)
            {
                var button = gamepad.TryGetChildControl<ButtonControl>(name);
                if (button != null && button.isPressed) return 1;
            }
            return 0;
        }

#if UNITY_STANDALONE_WIN || UNITY_EDITOR_WIN
        private static class WindowsXInput
        {
            private const uint ErrorSuccess = 0;
            private const ushort DpadUp = 0x0001;
            private const ushort DpadDown = 0x0002;
            private const ushort DpadLeft = 0x0004;
            private const ushort DpadRight = 0x0008;
            private const ushort Start = 0x0010;
            private const ushort Back = 0x0020;
            private const ushort LeftThumb = 0x0040;
            private const ushort RightThumb = 0x0080;
            private const ushort LeftShoulder = 0x0100;
            private const ushort RightShoulder = 0x0200;
            private const ushort A = 0x1000;
            private const ushort B = 0x2000;
            private const ushort X = 0x4000;
            private const ushort Y = 0x8000;
            private const float LeftStickDeadzone = 7849f / 32767f;
            private const float RightStickDeadzone = 8689f / 32767f;

            private static bool xinputAvailable = true;

            [StructLayout(LayoutKind.Sequential)]
            private struct XInputGamepad
            {
                public ushort Buttons;
                public byte LeftTrigger;
                public byte RightTrigger;
                public short ThumbLX;
                public short ThumbLY;
                public short ThumbRX;
                public short ThumbRY;
            }

            [StructLayout(LayoutKind.Sequential)]
            private struct XInputState
            {
                public uint PacketNumber;
                public XInputGamepad Gamepad;
            }

            [DllImport("xinput1_4.dll", EntryPoint = "XInputGetState")]
            private static extern uint XInputGetState(uint userIndex, out XInputState state);

            public static bool TryRead(out GamepadJoySample sample)
            {
                sample = null;
                if (!xinputAvailable) return false;

                try
                {
                    for (uint index = 0; index < 4; index++)
                    {
                        if (XInputGetState(index, out var state) != ErrorSuccess) continue;
                        sample = CreateSample(index, state.Gamepad);
                        return true;
                    }
                }
                catch (DllNotFoundException)
                {
                    xinputAvailable = false;
                }
                catch (EntryPointNotFoundException)
                {
                    xinputAvailable = false;
                }

                return false;
            }

            private static GamepadJoySample CreateSample(uint index, XInputGamepad gamepad)
            {
                var left = ApplyStickDeadzone(gamepad.ThumbLX, gamepad.ThumbLY, LeftStickDeadzone);
                var right = ApplyStickDeadzone(gamepad.ThumbRX, gamepad.ThumbRY, RightStickDeadzone);
                var axes = new float[8];
                axes[0] = -left.x;
                axes[1] = left.y;
                axes[2] = right.y;
                axes[3] = -right.x;
                axes[4] = TriggerAxis(gamepad.RightTrigger / 255f);
                axes[5] = TriggerAxis(gamepad.LeftTrigger / 255f);
                axes[6] = Has(gamepad.Buttons, DpadLeft) ? 1f : Has(gamepad.Buttons, DpadRight) ? -1f : 0f;
                axes[7] = Has(gamepad.Buttons, DpadUp) ? 1f : Has(gamepad.Buttons, DpadDown) ? -1f : 0f;

                var buttons = new int[15];
                buttons[0] = Button(gamepad.Buttons, A);
                buttons[1] = Button(gamepad.Buttons, B);
                buttons[3] = Button(gamepad.Buttons, X);
                buttons[4] = Button(gamepad.Buttons, Y);
                buttons[6] = Button(gamepad.Buttons, LeftShoulder);
                buttons[7] = Button(gamepad.Buttons, RightShoulder);
                buttons[10] = Button(gamepad.Buttons, Back);
                buttons[11] = Button(gamepad.Buttons, Start);
                // The Guide button is intentionally zero: XInputGetState does not expose it.
                buttons[13] = Button(gamepad.Buttons, LeftThumb);
                buttons[14] = Button(gamepad.Buttons, RightThumb);

                return new GamepadJoySample
                {
                    ControllerType = "xbox",
                    Description = $"XInput Controller {index + 1}",
                    DeviceId = 1000 + (int)index,
                    Axes = axes,
                    Buttons = buttons
                };
            }

            private static Vector2 ApplyStickDeadzone(short rawX, short rawY, float deadzone)
            {
                var value = new Vector2(NormalizeStick(rawX), NormalizeStick(rawY));
                var magnitude = value.magnitude;
                if (magnitude <= deadzone) return Vector2.zero;
                var scaledMagnitude = Mathf.Clamp01((magnitude - deadzone) / (1f - deadzone));
                return value / magnitude * scaledMagnitude;
            }

            private static float NormalizeStick(short value)
            {
                return value < 0 ? value / 32768f : value / 32767f;
            }

            private static bool Has(ushort value, ushort flag)
            {
                return (value & flag) != 0;
            }

            private static int Button(ushort value, ushort flag)
            {
                return Has(value, flag) ? 1 : 0;
            }
        }
#endif
    }
}
