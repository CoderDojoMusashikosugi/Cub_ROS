#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import threading
import subprocess
import asyncio
import time
"""
Compatibility shim for Python 3.10 + old websockets (<=9.x):
websockets 9.x passes loop=... to asyncio.Lock(), which raises on 3.10.
Monkey-patch asyncio.Lock to ignore the deprecated 'loop' kwarg.
"""
_orig_asyncio_Lock = asyncio.Lock
def _lock_no_loop_kwarg(*args, **kwargs):
    kwargs.pop('loop', None)
    return _orig_asyncio_Lock(*args, **kwargs)
asyncio.Lock = _lock_no_loop_kwarg  # type: ignore

# Also ignore 'loop' kwarg in asyncio.wait_for and asyncio.ensure_future
_orig_asyncio_wait_for = asyncio.wait_for
def _wait_for_no_loop_kwarg(awaitable, timeout, *args, **kwargs):
    kwargs.pop('loop', None)
    return _orig_asyncio_wait_for(awaitable, timeout, *args, **kwargs)
asyncio.wait_for = _wait_for_no_loop_kwarg  # type: ignore

_orig_asyncio_ensure_future = asyncio.ensure_future
def _ensure_future_no_loop_kwarg(coro_or_future, *args, **kwargs):
    kwargs.pop('loop', None)
    return _orig_asyncio_ensure_future(coro_or_future, *args, **kwargs)
asyncio.ensure_future = _ensure_future_no_loop_kwarg  # type: ignore

# asyncio.sleep
_orig_asyncio_sleep = asyncio.sleep
async def _sleep_no_loop_kwarg(delay, *args, **kwargs):
    kwargs.pop('loop', None)
    return await _orig_asyncio_sleep(delay, *args, **kwargs)
asyncio.sleep = _sleep_no_loop_kwarg  # type: ignore

import websockets
import json
import os
import signal
from flask import Flask, send_from_directory, Response, request
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import CompressedImage
from diagnostic_msgs.msg import DiagnosticArray
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

import logging
logging.basicConfig(level=logging.DEBUG)

# Global variables
rosbag_process = None
websocket_clients = set()
asyncio_loop = None
ws_port_actual = None
WS_PORT_DEFAULT = int(os.environ.get('HANDY2_WS_PORT', '8766'))

latest_colour_gains = None
diagnostics_lock = threading.Lock()

# Cached parameters
cached_params = {
    'AnalogueGain': None,
    'AwbEnable': None,
    'ColourGains': None,
    'shutter_on_us': None,
    'shutter_offset_us': None
}
params_lock = threading.Lock()

debug_info = {
    "image_count": 0,
    "diag_count": 0,
    "last_image_time": 0,
    "last_diag_time": 0,
    "streaming_enabled_bool": False
}
debug_lock = threading.Lock()

# Flask App
app = Flask(__name__)
web_dir = os.path.join(get_package_share_directory('handy2_bringup'), 'web')

@app.route('/debug/status')
def debug_status():
    with debug_lock:
        info = debug_info.copy()
    with streaming_lock:
        info["streaming_enabled_global"] = streaming_enabled
    return json.dumps(info), 200, {"Content-Type": "application/json"}

@app.route('/')
def index():
    return send_from_directory(web_dir, 'index.html')

@app.route('/camera_settings')
def camera_settings_page():
    return send_from_directory(web_dir, 'camera_settings.html')

@app.route('/<path:path>')
def send_web_files(path):
    return send_from_directory(web_dir, path)

def param_polling_thread():
    """Background thread to slowly poll parameters and update the cache without blocking the UI."""
    while True:
        try:
            updates = {}
            # Camera Node params
            for p in ['AnalogueGain', 'AwbEnable', 'ColourGains']:
                res = subprocess.run(['ros2', 'param', 'get', '/camera_node', p], capture_output=True, text=True)
                if res.returncode == 0:
                    line = res.stdout.strip()
                    if ':' in line:
                        val_part = line.split(':', 1)[-1].strip()
                        if "Double value" in line or "Double values" in line:
                            if '[' in val_part:
                                updates[p] = json.loads(val_part)
                            else:
                                updates[p] = float(val_part)
                        elif "Boolean value" in line:
                            updates[p] = val_part.lower() == 'true'
            
            # Pico Node params
            for p in ['shutter_on_us', 'shutter_offset_us']:
                res = subprocess.run(['ros2', 'param', 'get', '/pico_node', p], capture_output=True, text=True)
                if res.returncode == 0:
                    line = res.stdout.strip()
                    if ':' in line:
                        val_part = line.split(':', 1)[-1].strip()
                        if "Integer value" in line:
                            updates[p] = int(val_part)
                            
            with params_lock:
                cached_params.update(updates)
                
        except Exception as e:
            app.logger.error(f"Param polling error: {e}")
            
        time.sleep(5)  # Poll every 5 seconds

# Camera Parameter Endpoints
@app.route('/camera/get_params')
def get_camera_params():
    if not hasattr(app, 'ros_node'):
        return "ROS node not initialized", 500
    
    with params_lock:
        params_copy = cached_params.copy()
        
    with diagnostics_lock:
        params_copy['latest_colour_gains'] = latest_colour_gains
        
    return json.dumps(params_copy), 200, {"Content-Type": "application/json"}

@app.route('/camera/set_param', methods=['POST'])
def set_camera_param():
    data = request.json
    node = data.get('node')
    param = data.get('param')
    value = data.get('value')
    
    if not node or not param:
        return "Missing node or param", 400
        
    try:
        cmd = ['ros2', 'param', 'set', node, param]
        
        # Enforce type casting for known double parameters because JS sends whole numbers as ints
        # and ros2 param set will reject "100" for a double, requiring "100.0".
        if param in ['AnalogueGain'] and isinstance(value, (int, float)):
            value = float(value)
            
        if isinstance(value, bool):
            cmd.append(str(value).lower())
        elif isinstance(value, list):
            cmd.append(json.dumps(value))
        elif isinstance(value, float):
            # Ensure it always has a decimal point even if it's a whole number
            cmd.append(f"{value:.1f}" if value.is_integer() else str(value))
        else:
            cmd.append(str(value))
            
        res = subprocess.run(cmd, capture_output=True, text=True)
        
        # ros2 param set returns 0 even if it fails, so we must check stdout
        if res.returncode == 0 and "Setting parameter failed" not in res.stdout:
            # Optimistically update the cache so the UI sees it immediately
            with params_lock:
                cached_params[param] = value
            return "Param set successfully", 200
        else:
            error_msg = res.stdout if "Setting parameter failed" in res.stdout else res.stderr
            return error_msg, 500
    except Exception as e:
        return str(e), 500

@app.route('/camera/get_diagnostics')
def get_camera_diagnostics():
    with diagnostics_lock:
        return json.dumps({"ColourGains": latest_colour_gains}), 200, {"Content-Type": "application/json"}

# Video streaming
latest_jpeg_data = None
jpeg_lock = threading.Lock()
streaming_enabled = False  # Default: OFF
streaming_lock = threading.Lock()

def gen_frames():
    global latest_jpeg_data, streaming_enabled
    while True:
        with streaming_lock:
            enabled = streaming_enabled
        
        if not enabled:
            # Send a very small blank or "paused" frame would be better, but sleep for now
            import time
            time.sleep(0.5)
            continue
            
        frame = None
        with jpeg_lock:
            if latest_jpeg_data is not None:
                frame = latest_jpeg_data
        
        if frame:
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
            import time
            time.sleep(0.06) # Slightly slower to be safer (approx 15-18 fps)
        else:
            import time
            time.sleep(0.1)

@app.route('/video_feed')
def video_feed():
    # Automatically enable streaming when the feed is requested
    global streaming_enabled
    with streaming_lock:
        streaming_enabled = True
    if hasattr(app, 'ros_node'):
        app.ros_node.enable_streaming()
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/ws_port')
def get_ws_port():
    # Provide the actual port the WS server is listening on
    # so the frontend can connect even if the default port is busy.
    global ws_port_actual
    port = ws_port_actual if ws_port_actual is not None else WS_PORT_DEFAULT
    return json.dumps({"port": port}), 200, {"Content-Type": "application/json"}

@app.route('/streaming/status')
def streaming_status():
    global streaming_enabled
    with streaming_lock:
        return json.dumps({"enabled": streaming_enabled}), 200, {"Content-Type": "application/json"}

@app.route('/streaming/enable', methods=['POST'])
def enable_streaming():
    global streaming_enabled
    with streaming_lock:
        streaming_enabled = True
    # Notify the ROS node to start subscription
    if hasattr(app, 'ros_node'):
        app.ros_node.enable_streaming()
    return json.dumps({"enabled": True}), 200, {"Content-Type": "application/json"}

@app.route('/streaming/disable', methods=['POST'])
def disable_streaming():
    global streaming_enabled
    with streaming_lock:
        streaming_enabled = False
    # Notify the ROS node to stop subscription
    if hasattr(app, 'ros_node'):
        app.ros_node.disable_streaming()
    return json.dumps({"enabled": False}), 200, {"Content-Type": "application/json"}

@app.route('/start_rosbag')
def start_rosbag():
    global rosbag_process
    if rosbag_process and rosbag_process.poll() is None:
        return "Rosbag is already running.", 400
    
    if asyncio_loop:
        asyncio.run_coroutine_threadsafe(broadcast_message("> Starting rosbag recording...\n"), asyncio_loop)
    
    try:
        # Start Rosbag process
        command = ['ros2', 'launch', 'handy2_bringup', 'rosbag.launch.py']
        rosbag_process = subprocess.Popen(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
            universal_newlines=True
        )
        
        # Start streaming output
        output_thread = threading.Thread(target=read_process_output, args=(rosbag_process,))
        output_thread.daemon = True
        output_thread.start()
        
        return "Rosbag recording started.", 200
    except Exception as e:
        error_msg = f"Failed to start rosbag: {e}\n"
        if asyncio_loop:
            asyncio.run_coroutine_threadsafe(broadcast_message(error_msg), asyncio_loop)
        return error_msg, 500


@app.route('/stop_rosbag')
def stop_rosbag():
    global rosbag_process

    
    # Stop rosbag process
    if rosbag_process and rosbag_process.poll() is None:
        if asyncio_loop:
            asyncio.run_coroutine_threadsafe(broadcast_message("> Stopping rosbag recording...\n"), asyncio_loop)
        
        os.kill(rosbag_process.pid, signal.SIGINT)
        try:
            rosbag_process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.kill(rosbag_process.pid, signal.SIGKILL)
        
        rosbag_process = None
        if asyncio_loop:
            asyncio.run_coroutine_threadsafe(broadcast_message("> Rosbag recording stopped.\n"), asyncio_loop)
    else:
        # If not running, just ensure cleanup
        rosbag_process = None
    
    return "Rosbag recording stopped.", 200

@app.route('/shutdown')
def shutdown_host():
    if asyncio_loop:
        message = "> Received shutdown command. Attempting to power off the host...\n"
        asyncio.run_coroutine_threadsafe(broadcast_message(message), asyncio_loop)
    
    try:
        # Command to shutdown the host via dbus
        command = [
            'sudo', 'dbus-send', '--system', '--print-reply', 
            '--dest=org.freedesktop.login1', 
            '/org/freedesktop/login1', 
            'org.freedesktop.login1.Manager.PowerOff', 'boolean:true'
        ]
        result = subprocess.run(command, capture_output=True, text=True, timeout=10)
        
        if result.returncode == 0:
            return "Shutdown command sent successfully.", 200
        else:
            error_message = f"Failed to send shutdown command. Error: {result.stderr}"
            if asyncio_loop:
                asyncio.run_coroutine_threadsafe(broadcast_message(error_message + "\n"), asyncio_loop)
            return error_message, 500
            
    except Exception as e:
        error_msg = f"Exception while sending shutdown command: {e}\n"
        if asyncio_loop:
            asyncio.run_coroutine_threadsafe(broadcast_message(error_msg), asyncio_loop)
        return error_msg, 500


def read_process_output(process):
    global asyncio_loop
    if process.stdout:
        for line in iter(process.stdout.readline, ''):
            if asyncio_loop:
                asyncio.run_coroutine_threadsafe(broadcast_message(line), asyncio_loop)
        process.stdout.close()

# WebSocket Server
async def websocket_handler(websocket, path=None):
    websocket_clients.add(websocket)
    try:
        await websocket.wait_closed()
    finally:
        websocket_clients.remove(websocket)

async def broadcast_message(message):
    if websocket_clients:
        # Send to each client independently to avoid one failure breaking all.
        coros = []
        for client in list(websocket_clients):
            async def _send(c=client):
                try:
                    await c.send(message)
                except Exception:
                    # Drop broken client
                    try:
                        websocket_clients.remove(c)
                    except KeyError:
                        pass
            coros.append(_send())
        if coros:
            await asyncio.gather(*coros, return_exceptions=True)

def run_asyncio_loop():
    global asyncio_loop
    global ws_port_actual
    asyncio_loop = asyncio.new_event_loop()
    asyncio.set_event_loop(asyncio_loop)

    async def main_async():
        # Start WebSocket server with port auto-selection in case of conflicts
        port = WS_PORT_DEFAULT
        last_exc = None
        for _ in range(20):  # try a range of ports
            try:
                # host=None binds to all interfaces (IPv4 and IPv6)
                async with websockets.serve(websocket_handler, None, port):
                    ws_port_actual = port
                    # Informatively print to stdout for ros2 logs
                    print(f"[web_control_node] WebSocket server bound on ws://*:{port}")
                    await asyncio.Future()  # run forever
            except OSError as e:
                last_exc = e
                port += 1
                continue
        # If we get here, all attempts failed
        print(f"[web_control_node] Failed to bind WebSocket server starting at {WS_PORT_DEFAULT}: {last_exc}")
        raise last_exc
    
    asyncio_loop.run_until_complete(main_async())


# ROS2 Node
class WebControlNode(Node):
    def __init__(self):
        super().__init__('web_control_node')
        self.get_logger().info('Web Control Node has been started.')
        
        # Run Flask in a separate thread
        flask_thread = threading.Thread(target=lambda: app.run(host='0.0.0.0', port=5000, debug=False))
        flask_thread.daemon = True
        flask_thread.start()
        self.get_logger().info('Flask server is running on http://0.0.0.0:5000')

        # Run WebSocket server in a separate thread
        ws_thread = threading.Thread(target=run_asyncio_loop)
        ws_thread.daemon = True
        ws_thread.start()
        self.get_logger().info('WebSocket server thread started (dynamic port). Open /ws_port for actual port.')

        # Run Parameter Polling thread
        param_thread = threading.Thread(target=param_polling_thread)
        param_thread.daemon = True
        param_thread.start()
        self.get_logger().info('Parameter polling thread started.')

        # Image subscription (initially None, created on demand)
        self._image_sub = None
        
        # Diagnostics subscription
        self._diagnostics_sub = self.create_subscription(
            DiagnosticArray,
            '/diagnostics',
            self._on_diagnostics,
            10
        )
        
        # Register this node with the Flask app for streaming control
        app.ros_node = self

    def _on_diagnostics(self, msg):
        global latest_colour_gains
        with debug_lock:
            debug_info["diag_count"] += 1
            debug_info["last_diag_time"] = time.time()
            
        for status in msg.status:
            is_camera = "imx296" in status.hardware_id or any(v.key == "ColourGains" for v in status.values)
            if is_camera:
                for val in status.values:
                    if val.key == "ColourGains":
                        try:
                            gains_str = val.value.replace('[', '').replace(']', '').replace(',', ' ')
                            gains = [float(x) for x in gains_str.split()]
                            if len(gains) == 2:
                                with diagnostics_lock:
                                    latest_colour_gains = gains
                        except Exception as e:
                            self.get_logger().error(f"Failed to parse ColourGains: {e}")

    def enable_streaming(self):
        """Enable image streaming by creating subscription."""
        with debug_lock:
            debug_info["streaming_enabled_bool"] = True
        if self._image_sub is None:
            # ... (rest unchanged)
            self._image_sub = self.create_subscription(
                CompressedImage,
                '/camera_node/image_raw/compressed',
                self._on_image_compressed,
                10
            )
            self.get_logger().info('Image streaming enabled.')
    
    def disable_streaming(self):
        """Disable image streaming by destroying subscription."""
        if self._image_sub is not None:
            self.destroy_subscription(self._image_sub)
            self._image_sub = None
            global latest_jpeg_data
            with jpeg_lock:
                latest_jpeg_data = None
            self.get_logger().info('Image streaming disabled.')
    
    def _on_image_compressed(self, msg):
        global latest_jpeg_data
        with debug_lock:
            debug_info["image_count"] += 1
            debug_info["last_image_time"] = time.time()
        with jpeg_lock:
            latest_jpeg_data = msg.data


def main(args=None):
    rclpy.init(args=args)
    node = WebControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up the rosbag process when the node is shut down
        global rosbag_process
        if rosbag_process and rosbag_process.poll() is None:
            node.get_logger().info('Shutting down rosbag process...')
            os.kill(rosbag_process.pid, signal.SIGINT)
            rosbag_process.wait()

        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()