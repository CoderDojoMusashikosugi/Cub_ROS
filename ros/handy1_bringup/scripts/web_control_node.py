#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import threading
import subprocess
import asyncio
import websockets
import os
import signal
from flask import Flask, send_from_directory
from ament_index_python.packages import get_package_share_directory

# Global variables
rosbag_process = None
websocket_clients = set()
asyncio_loop = None

# Flask App
app = Flask(__name__)
web_dir = os.path.join(get_package_share_directory('handy1_bringup'), 'web')

@app.route('/')
def index():
    return send_from_directory(web_dir, 'index.html')

@app.route('/<path:path>')
def send_web_files(path):
    return send_from_directory(web_dir, path)

@app.route('/start_rosbag')
def start_rosbag():
    global rosbag_process
    if rosbag_process and rosbag_process.poll() is None:
        return "Rosbag is already running.", 400
    
    if asyncio_loop:
        asyncio.run_coroutine_threadsafe(broadcast_message("> Starting rosbag recording...\n"), asyncio_loop)
    
    try:
        # Using ros2 launch command
        command = ['ros2', 'launch', 'handy1_bringup', 'rosbag.launch.py']
        rosbag_process = subprocess.Popen(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
            universal_newlines=True
        )
        
        # Thread to read output and broadcast
        output_thread = threading.Thread(target=read_process_output, args=(rosbag_process, asyncio_loop))
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
    if rosbag_process and rosbag_process.poll() is None:
        if asyncio_loop:
            asyncio.run_coroutine_threadsafe(broadcast_message("> Stopping rosbag recording...\n"), asyncio_loop)
        
        # Send SIGINT to the process group to gracefully stop rosbag
        os.kill(rosbag_process.pid, signal.SIGINT)
        
        try:
            rosbag_process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            if asyncio_loop:
                asyncio.run_coroutine_threadsafe(broadcast_message("> Rosbag did not terminate gracefully. Forcing kill...\n"), asyncio_loop)
            os.kill(rosbag_process.pid, signal.SIGINT) # Try again with SIGINT
            try:
                rosbag_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.kill(rosbag_process.pid, signal.SIGKILL) # Force kill
        
        rosbag_process = None
        if asyncio_loop:
            asyncio.run_coroutine_threadsafe(broadcast_message("> Rosbag recording stopped.\n"), asyncio_loop)
        return "Rosbag recording stopped.", 200
    else:
        return "Rosbag is not running.", 400

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


def read_process_output(process, loop):
    if process.stdout:
        for line in iter(process.stdout.readline, ''):
            if loop:
                asyncio.run_coroutine_threadsafe(broadcast_message(line), loop)
        process.stdout.close()

# WebSocket Server
async def websocket_handler(websocket):
    websocket_clients.add(websocket)
    try:
        await websocket.wait_closed()
    finally:
        websocket_clients.remove(websocket)

async def broadcast_message(message):
    if websocket_clients:
        await asyncio.gather(
            *[client.send(message) for client in websocket_clients],
            return_exceptions=False,
        )

def run_asyncio_loop():
    global asyncio_loop
    asyncio_loop = asyncio.new_event_loop()
    asyncio.set_event_loop(asyncio_loop)

    async def main_async():
        # Start WebSocket server
        async with websockets.serve(websocket_handler, "0.0.0.0", 8765):
            await asyncio.Future()  # run forever
    
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
        self.get_logger().info('WebSocket server is running on ws://0.0.0.0:8765')

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
