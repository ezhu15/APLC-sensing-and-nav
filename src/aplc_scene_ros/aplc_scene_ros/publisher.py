"""Static portable map visualization; no Quest connection or pose alignment."""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from aplc_scene.map import load_map
from .geometry import triangles, room_center


class ScenePublisher(Node):
    def __init__(self):
        super().__init__('scene_publisher')
        self.declare_parameter('map_file','')
        self.declare_parameter('show_ceilings',False)
        self.declare_parameter('show_labels',True)
        self.declare_parameter('republish_seconds',5.0)
        self.data = load_map(self.get_parameter('map_file').value)
        self.show_ceilings = self.get_parameter('show_ceilings').value
        self.show_labels = self.get_parameter('show_labels').value
        period = self.get_parameter('republish_seconds').value
        if period <= 0: raise ValueError('republish_seconds must be positive')
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(MarkerArray,'scene/markers',qos)
        self.markers = self.make_markers()
        self.publish_scene()
        self.timer = self.create_timer(period,self.publish_scene)
        self.get_logger().info(f"Map {self.data['map_id']}: {len(self.data['rooms'])} rooms, "
                               f"{len(self.data['anchors'])} anchors, frame={self.data['frame_id']}; "
                               f"{len(self.markers.markers)-1} markers. Alignment: {self.data['alignment']['status']}")

    def marker(self,namespace,index,kind):
        m = Marker()
        m.header.frame_id = self.data['frame_id']
        m.ns = namespace; m.id = index; m.type = kind; m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 1.0
        m.color.r = 0.65; m.color.g = 0.7; m.color.b = 0.8; m.color.a = 0.75
        return m

    def make_markers(self):
        result = MarkerArray()
        clear = self.marker('scene',0,Marker.TRIANGLE_LIST)
        clear.action = Marker.DELETEALL
        result.markers.append(clear)
        anchors = {a['uuid']:a for a in self.data['anchors']}
        for index,a in enumerate(sorted(anchors.values(),key=lambda a:a['uuid'])):
            labels = set(a['labels'])
            if 'CEILING' in labels and not self.show_ceilings: continue
            points = triangles(a['geometry'])
            if points:
                m = self.marker('anchors',index,Marker.TRIANGLE_LIST)
                m.points = [Point(x=float(p[0]),y=float(p[1]),z=float(p[2])) for p in points]
                if 'FLOOR' in labels: m.color.r,m.color.g,m.color.b = 0.35,0.6,0.4
                if labels & {'WALL_ART','WINDOW_FRAME','DOOR_FRAME','OPENING'}:
                    m.color.r,m.color.g,m.color.b,m.color.a = 1.0,0.5,0.1,1.0
                result.markers.append(m)
            if self.show_labels and labels & {'WALL_ART','WINDOW_FRAME'}:
                m = self.marker('landmark_labels',index,Marker.TEXT_VIEW_FACING)
                m.text = ', '.join(a['labels'])+' '+a['uuid'][:8]
                p = a['pose']['position_m']; m.pose.position = Point(x=float(p[0]),y=float(p[1]),z=float(p[2])+0.15)
                m.scale.z = 0.15; m.color.a = 1.0
                result.markers.append(m)
        if self.show_labels:
            for index,room in enumerate(sorted(self.data['rooms'],key=lambda r:r['uuid'])):
                p = room_center(room,anchors)
                if p is None: continue
                m = self.marker('rooms',index,Marker.TEXT_VIEW_FACING)
                m.text = room.get('name') or 'Room '+room['uuid'][:8]
                m.pose.position = Point(x=float(p[0]),y=float(p[1]),z=float(p[2])+0.25)
                m.scale.z = 0.25; m.color.r=m.color.g=m.color.b=m.color.a=1.0
                result.markers.append(m)
        return result

    def publish_scene(self):
        # Zero timestamp: geometry is static in the fixed frame; no TF lookup at capture time.
        self.publisher.publish(self.markers)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ScenePublisher()
        rclpy.spin(node)
    except (ValueError,KeyError,TypeError,OSError) as exc:
        rclpy.logging.get_logger('scene_publisher').error(f'Scene load failed: {exc}')
        raise
    finally:
        if node is not None: node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
