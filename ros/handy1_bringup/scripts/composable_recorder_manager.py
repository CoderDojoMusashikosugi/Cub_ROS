#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from composition_interfaces.srv import LoadNode, UnloadNode
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
import sys
import signal
import time
import os
import yaml

class RecorderManager(Node):
    def __init__(self):
        super().__init__('rosbag_recorder_manager')
        self.container_name = 'handy1_driver_container'
        self.node_name = 'rosbag_recorder'
        self.unique_id = None
        
        self.load_client = self.create_client(
            LoadNode, 
            f'/{self.container_name}/_container/load_node'
        )
        self.unload_client = self.create_client(
            UnloadNode, 
            f'/{self.container_name}/_container/unload_node'
        )

    def load_recorder(self, bag_name, topics, storage_id='mcap', max_cache_size=536870912):
        while not self.load_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'Waiting for service {self.load_client.srv_name}...')
            
        request = LoadNode.Request()
        request.package_name = 'rosbag2_composable_recorder'
        request.plugin_name = 'rosbag2_composable_recorder::ComposableRecorder'
        request.node_name = self.node_name
        request.node_namespace = ''
        
        # Setup parameters
        params = []
        
        # bag_name
        p_bag = Parameter()
        p_bag.name = 'bag_name'
        p_bag.value = ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=bag_name)
        params.append(p_bag)
        
        # storage_id
        p_storage = Parameter()
        p_storage.name = 'storage_id'
        p_storage.value = ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=storage_id)
        params.append(p_storage)

        # max_cache_size
        p_cache = Parameter()
        p_cache.name = 'max_cache_size'
        p_cache.value = ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=max_cache_size)
        params.append(p_cache)

        # start_recording_immediately
        p_start = Parameter()
        p_start.name = 'start_recording_immediately'
        p_start.value = ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=True)
        params.append(p_start)

        # topics
        p_topics = Parameter()
        p_topics.name = 'topics'
        p_topics.value = ParameterValue(type=ParameterType.PARAMETER_STRING_ARRAY, string_array_value=topics)
        params.append(p_topics)

        request.parameters = params
        request.extra_arguments = [
            Parameter(name='use_intra_process_comms', value=ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=True))
        ]

        future = self.load_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None and future.result().success:
            self.unique_id = future.result().unique_id
            self.get_logger().info(f'Successfully loaded rosbag_recorder (ID: {self.unique_id}) to {self.container_name}')
            self.get_logger().info(f'Recording to: {bag_name}')
            return True
        else:
            self.get_logger().error('Failed to load rosbag_recorder')
            return False

    def unload_recorder(self):
        if self.unique_id is None:
            return

        self.get_logger().info('Unloading rosbag_recorder...')
        request = UnloadNode.Request()
        request.unique_id = self.unique_id
        
        future = self.unload_client.call_async(request)
        # Note: We can't spin indefinitely here because we are likely in shutdown context
        # We try to spin a bit to process the request
        timeout_sec = 2.0
        start = time.time()
        while rclpy.ok() and (time.time() - start) < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.1)
            if future.done():
                break

        if future.done() and future.result() is not None and future.result().success:
            self.get_logger().info('Successfully unloaded rosbag_recorder')
        else:
            self.get_logger().warn('Failed to verify unload completion (might have timed out or failed)')

def main():
    rclpy.init()
    
    # Retrieve arguments from environment variables passed by launch
    bag_name = os.environ.get('ROSBAG_NAME', 'rosbag_default')
    topics_str = os.environ.get('ROSBAG_TOPICS', '')
    topics = topics_str.split(',') if topics_str else []
    
    manager = RecorderManager()
    
    if manager.load_recorder(bag_name, topics):
        try:
            # Main loop
            while rclpy.ok():
                time.sleep(0.1)
        except KeyboardInterrupt:
            pass
        finally:
            manager.unload_recorder()
            manager.destroy_node()
            rclpy.shutdown()
    else:
        manager.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
