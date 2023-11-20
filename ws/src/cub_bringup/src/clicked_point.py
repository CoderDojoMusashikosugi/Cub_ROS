#!/usr/bin/env python3
import sys, termios, atexit
from select import select

import rospy
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from tf.transformations import quaternion_from_euler
import yaml 


# save the terminal settings
fd = sys.stdin.fileno()
new_term = termios.tcgetattr(fd)
old_term = termios.tcgetattr(fd)

# new terminal setting unbuffered
new_term[3] = (new_term[3] & ~termios.ICANON & ~termios.ECHO)

# switch to normal terminal
def set_normal_term():
    termios.tcsetattr(fd, termios.TCSAFLUSH, old_term)

# switch to unbuffered terminal
def set_curses_term():
    termios.tcsetattr(fd, termios.TCSAFLUSH, new_term)

def putch(ch):
    sys.stdout.write(ch)

def getch():
    return sys.stdin.read(1)

def getche():
    ch = getch()
    putch(ch)
    return ch

def kbhit():
    dr,dw,de = select([sys.stdin], [], [], 0)
    return dr != []

def setup_kbhit():
    atexit.register(set_normal_term)
    set_curses_term()

import yaml

class MarkerPublisher:
    def __init__(self):
        # ROSノードの初期化
        rospy.init_node('marker_publisher_node', anonymous=True)

        # MarkerArrayメッセージの初期化
        self.cube_marker_array = MarkerArray()
        self.id_marker_array = MarkerArray()


        # Markerを配信するためのPublisher
        self.cube_marker_pub = rospy.Publisher('cube_marker_topic', MarkerArray, queue_size=10)
        self.id_marker_pub = rospy.Publisher('id_marker_topic', MarkerArray, queue_size=10)

        # geometry_msgs/PointStampedメッセージを購読
        rospy.Subscriber("/clicked_point", PointStamped, self.callback)
        self.changeId = None
        self.id_marker_array,self.cube_marker_array = self.load_from_yaml("test.yaml")
        self.updateData()

    def callback(self, data):
        newId = len(self.id_marker_array.markers)
        if self.changeId != None:
            print("changing position of id [" + str(self.changeId) + "]")
            newId = self.changeId
        print(newId)

        # 新しいCube Markerを作成
        cube_marker = Marker()
        cube_marker.header.frame_id = "map"  # RViz内の表示フレーム
        cube_marker.type = Marker.CUBE
        cube_marker.action = Marker.ADD
        cube_marker.scale.x = 0.3
        cube_marker.scale.y = 0.3
        cube_marker.scale.z = 0.3
        cube_marker.scale.x = 1.0
        cube_marker.scale.y = 1.0
        cube_marker.scale.z = 1.0
        cube_marker.color.r = 1.0
        cube_marker.color.g = 0.0
        cube_marker.color.b = 0.0
        cube_marker.color.a = 0.7
        cube_marker.pose.orientation.w = 1.0
        cube_marker.header.stamp = rospy.Time.now()
        cube_marker.pose.position = data.point
        cube_marker.id = newId

        # 新しいID Markerを作成
        id_marker = Marker()
        id_marker.header.frame_id = "map"  # RViz内の表示フレーム
        id_marker.type = Marker.TEXT_VIEW_FACING
        id_marker.action = Marker.ADD
        id_marker.text = str(newId)  # idの番号を表示  # idの番号を表示
        id_marker.scale.z = 0.4
        id_marker.scale.z = 1.2
        id_marker.color.r = 1.0
        id_marker.color.g = 1.0
        id_marker.color.b = 1.0
        id_marker.color.a = 0.7
        id_marker.pose.orientation.w = 1.0
        id_marker.header.stamp = rospy.Time.now()
        id_marker.pose.position = data.point
        id_marker.id = newId

        if self.changeId == None:
            # Cube MarkerをCube MarkerArrayに追加
            self.cube_marker_array.markers.append(cube_marker)
            # ID MarkerをID MarkerArrayに追加
            self.id_marker_array.markers.append(id_marker)
        else:
            self.cube_marker_array.markers[self.changeId] = cube_marker
            self.id_marker_array.markers[self.changeId] = id_marker

        # # 全てのMarkerの番号を更新
        # for i, marker in enumerate(self.cube_marker_array.markers):
        #     marker.id = i
        # # 全てのMarkerの番号を更新
        # for i, marker in enumerate(self.id_marker_array.markers):
        #     marker.id = i

        # Cube MarkerArrayを配信
        # self.cube_marker_pub.publish(self.cube_marker_array)
        # ID MarkerArrayを配信
        # self.id_marker_pub.publish(self.id_marker_array)
        self.updateData()

        self.save_to_yaml(self.id_marker_array, "test.yaml")
        self.changeId = None

    def updateData(self):
        # Cube MarkerArrayを配信
        self.cube_marker_pub.publish(self.cube_marker_array)
        # ID MarkerArrayを配信
        self.id_marker_pub.publish(self.id_marker_array)
        

    def save_to_yaml(self,id_marker_array, yaml_file_path):
        waypoints_data = []

        for marker in id_marker_array.markers:
            waypoint = {
                'id': marker.id,
                'x': marker.pose.position.x,
                'y': marker.pose.position.y,
            }
            waypoints_data.append(waypoint)

        data = {'waypoints_list': waypoints_data}

        with open(yaml_file_path, 'w') as yaml_file:
            yaml.dump(data, yaml_file, default_flow_style=False)


    def load_from_yaml(self, yaml_file_path):
        with open(yaml_file_path, 'r') as yaml_file:
            data = yaml.load(yaml_file, Loader=yaml.FullLoader)

        id_marker_array = MarkerArray()
        cube_marker_array = MarkerArray()

        i = 0

        for waypoint_data in data.get('waypoints_list', []):
            # ID Markerの作成
            id_marker = Marker()
            id_marker.header.frame_id = "map"
            id_marker.type = Marker.TEXT_VIEW_FACING
            id_marker.action = Marker.ADD
            id_marker.text = str(waypoint_data.get('id', ''))
            id_marker.scale.z = 1.2
            id_marker.color.r = 1.0
            id_marker.color.g = 0.0
            id_marker.color.b = 0.0
            id_marker.color.a = 0.7
            id_marker.pose.orientation.w = 1.0
            id_marker.header.stamp = rospy.Time.now()
            id_marker.pose.position.x = waypoint_data.get('x', 0.0)
            id_marker.pose.position.y = waypoint_data.get('y', 0.0)
            id_marker.id = i

            # Cube Markerの作成
            cube_marker = Marker()
            cube_marker.header.frame_id = "map"
            cube_marker.type = Marker.CUBE
            cube_marker.action = Marker.ADD
            cube_marker.scale.x = 1.0
            cube_marker.scale.y = 1.0
            cube_marker.scale.z = 1.0
            cube_marker.color.r = 1.0
            cube_marker.color.g = 0.0
            cube_marker.color.b = 0.0
            cube_marker.color.a = 0.7
            cube_marker.pose.orientation.w = 1.0
            cube_marker.header.stamp = rospy.Time.now()
            cube_marker.pose.position.x = waypoint_data.get('x', 0.0)
            cube_marker.pose.position.y = waypoint_data.get('y', 0.0)
            cube_marker.id = i

            id_marker_array.markers.append(id_marker)
            cube_marker_array.markers.append(cube_marker)
            i+=1

        return id_marker_array, cube_marker_array


    def movePoint(self,id):
        self.changeId = id


if __name__ == '__main__':
    print("usage")
    print("  1, launch rviz and this script and empty_map.py")
    print("  2, add following rviz visualization, /empty_occupancy_grid /cube_marker_topic /id_marker_topic ")
    print("  3, choose publish point tool and press 3d view")
    print("  4, global path yaml file will be cleated in current directory")
    

    setup_kbhit()
    strbuf = ""
    try:
        marker_publisher = MarkerPublisher()
        while not rospy.is_shutdown():
            if kbhit():
                ch = getch()
                print(ch,end="", flush=True)
                strbuf += ch
                if ch == '\n':
                    #print(" [enter]")
                    #print(strbuf)
                    try:
                        data = int(strbuf)
                        print("movePoint request num: "+str(data))
                        marker_publisher.movePoint(data)
                    except ValueError:
                        marker_publisher.movePoint(None)
                        print("clear movePoint request")
                        print("input number and press enter")
                    strbuf=""
            # rospy.spin_once()
        # rospy.spin()
    except rospy.ROSInterruptException:
        pass
