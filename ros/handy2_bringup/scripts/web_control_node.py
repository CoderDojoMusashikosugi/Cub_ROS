#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import threading
import subprocess
import asyncio
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
from flask import Flask, send_from_directory, Response
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import Int32
from sensor_msgs.msg import CompressedImage

# Global variables
rosbag_process = None
log_tail_process = None
websocket_clients = set()
asyncio_loop = None
ws_port_actual = None
WS_PORT_DEFAULT = int(os.environ.get('HANDY2_WS_PORT', '8766'))
# Latest sync status label (e.g., "NO", "PTP", "GPS") for initial push to clients
latest_sync_label = None

# Flask App
app = Flask(__name__)
web_dir = os.path.join(get_package_share_directory('handy2_bringup'), 'web')

@app.route('/')
def index():
    return send_from_directory(web_dir, 'index.html')

@app.route('/<path:path>')
def send_web_files(path):
    return send_from_directory(web_dir, path)

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
            # Send a blank frame when streaming is disabled
            import time
            time.sleep(0.1)
            continue
            
        with jpeg_lock:
            if latest_jpeg_data is not None:
                frame = latest_jpeg_data
            else:
                frame = None
        
        if frame:
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        else:
            pass
            
        import time
        time.sleep(0.04) # Limit to approx 25 fps

@app.route('/video_feed')
def video_feed():
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
    global rosbag_process, log_tail_process
    if rosbag_process and rosbag_process.poll() is None:
        return "Rosbag is already running.", 400
    
    if asyncio_loop:
        asyncio.run_coroutine_threadsafe(broadcast_message("> Starting rosbag recording...\n"), asyncio_loop)
    
    try:
        # Start Rosbag process
        command = ['ros2', 'launch', 'handy2_bringup', 'rosbag.launch.py']
        rosbag_process = subprocess.Popen(
            command,
            stdout=subprocess.PIPE, # We don't stream this anymore, but capturing avoids noise
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
            universal_newlines=True
        )
        
        return "Rosbag recording started.", 200
    except Exception as e:
        error_msg = f"Failed to start rosbag: {e}\n"
        if asyncio_loop:
            asyncio.run_coroutine_threadsafe(broadcast_message(error_msg), asyncio_loop)
        return error_msg, 500


@app.route('/stop_rosbag')
def stop_rosbag():
    global rosbag_process, log_tail_process
    
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

def start_log_tailing():
    global log_tail_process
    if log_tail_process and log_tail_process.poll() is None:
        return
    
    try:
        # Ensure log file exists
        subprocess.run(['touch', '/home/cub/launch.log'])

        # Start tail process for logs
        tail_cmd = ['tail', '-f', '/home/cub/launch.log']
        log_tail_process = subprocess.Popen(
            tail_cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
            universal_newlines=True
        )

        # Thread to read TAIL output and broadcast
        output_thread = threading.Thread(target=read_process_output, args=(log_tail_process,))
        output_thread.daemon = True
        output_thread.start()
    except Exception as e:
        print(f"Failed to start log tailing: {e}")

# WebSocket Server
async def websocket_handler(websocket, path=None):
    websocket_clients.add(websocket)
    try:
        # Send initial sync status to newly connected client
        if latest_sync_label is not None:
            try:
                await websocket.send(f"SYNC_STATUS::{latest_sync_label}")
            except Exception:
                pass
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

        # Start log tailing
        start_log_tailing()
        
        # Subscribe to livox/time_type to monitor sync state
        self._last_time_type = None
        self._time_type_sub = self.create_subscription(Int32, 'livox/time_type', self._on_time_type, 10)

        # Image subscription (initially None, created on demand)
        self._image_sub = None
        
        # Register this node with the Flask app for streaming control
        app.ros_node = self

    def enable_streaming(self):
        """Enable image streaming by creating subscription."""
        if self._image_sub is None:
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
        with jpeg_lock:
            latest_jpeg_data = msg.data

    def _on_time_type(self, msg: Int32):
        global latest_sync_label
        value = int(msg.data)
        if value == 0:
            label = 'NO'
        elif value == 1:
            label = 'PTP'
        elif value == 2:
            label = 'GPS'
        else:
            label = f'UNKNOWN({value})'

        # Only broadcast when changed
        if label != latest_sync_label:
            latest_sync_label = label
            self.get_logger().info(f'Livox Sync state: {label}')
            if asyncio_loop:
                asyncio.run_coroutine_threadsafe(
                    broadcast_message(f"SYNC_STATUS::{label}"),
                    asyncio_loop
                )
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
        
        # Clean up log tail process
        global log_tail_process
        if log_tail_process and log_tail_process.poll() is None:
            node.get_logger().info('Shutting down log tail process...')
            os.kill(log_tail_process.pid, signal.SIGTERM)
            log_tail_process.wait()

        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()