import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rmf_fleet_msgs.msg import FleetState
from rmf_building_map_msgs.msg import BuildingMap
from rmf_site_map_msgs.msg import SiteMap
from visualization_msgs.msg import MarkerArray, Marker
import threading
import flask
from flask import Flask, render_template
from flask_socketio import SocketIO
import json
import os
import base64
import time

# Initialize Flask and SocketIO
# Use absolute paths for templates/static based on this file's location
base_dir = os.path.abspath(os.path.dirname(__file__))
template_dir = os.path.join(base_dir, 'templates')
static_dir = os.path.join(base_dir, 'static')

app = Flask(__name__, template_folder=template_dir, static_folder=static_dir)
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading')

# Global storage for latest states
latest_fleet_states = {}
latest_building_map = None
# Schedule markers cache: { ns: { id: marker_data } }
schedule_markers_cache = {}

class RmfWebVizNode(Node):
    def __init__(self):
        super().__init__('rmf_web_viz')
        
        # QoS Profile for Fleet States (usually Volatile is fine, but Best Effort is common for high freq)
        # Using default reliable for now or system default.
        self.create_subscription(
            FleetState,
            '/fleet_states',
            self.fleet_state_callback,
            10
        )
        
        # QoS Profile for Building Map (Transient Local is required to get the map if it was published before node start)
        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        
        # Subscribe to building map (floorplan)
        self.create_subscription(
            BuildingMap,
            '/map',
            self.building_map_callback,
            qos_profile=map_qos
        )

        # Subscribe to schedule markers
        self.create_subscription(
            MarkerArray,
            '/schedule_markers',
            self.schedule_markers_callback,
            10
        )

        self.get_logger().info('RMF Web Viz Node Started')

    def fleet_state_callback(self, msg):
        global latest_fleet_states
        
        # Convert ROS message to dict for JSON serialization
        fleet_data = {
            'name': msg.name,
            'robots': []
        }
        
        for robot in msg.robots:
            robot_data = {
                'name': robot.name,
                'model': robot.model,
                'battery_percent': robot.battery_percent,
                'location': {
                    'x': robot.location.x,
                    'y': robot.location.y,
                    'yaw': robot.location.yaw,
                    'level_name': robot.location.level_name,
                },
                'mode': robot.mode.mode,
                'task_id': robot.task_id
            }
            fleet_data['robots'].append(robot_data)
        
        latest_fleet_states[msg.name] = fleet_data
        
        # Emit update via SocketIO
        socketio.emit('fleet_update', fleet_data)

    def building_map_callback(self, msg):
        global latest_building_map
        if latest_building_map is not None:
            return  # Only need to load map once usually, or handle updates if needed
            
        self.get_logger().info(f'Received building map: {msg.name}')
        
        map_data = {
            'name': msg.name,
            'levels': []
        }
        
        for level in msg.levels:
            # Debug: print available fields
            # self.get_logger().info(f"Level fields: {dir(level)}")
            
            level_data = {
                'name': level.name,
                'elevation': level.elevation,
                'images': []
            }
            
            for image in level.images:
                # Convert image data to base64 for frontend display
                img_base64 = base64.b64encode(bytes(image.data)).decode('utf-8')
                
                level_data['images'].append({
                    'name': image.name,
                    'x': image.x_offset,
                    'y': image.y_offset,
                    'scale': image.scale, 
                    'encoding': image.encoding,
                    'data_base64': img_base64,
                    'data_len': len(image.data)
                })
            
            # Extract data from graphs (nav_graphs and wall_graph)
            # Structure: Level -> nav_graphs (list) -> Graph -> vertices/edges
            #            Level -> wall_graph (Graph) -> vertices/edges
            
            graphs = []
            
            # Process Navigation Graphs
            for graph in getattr(level, 'nav_graphs', []):
                graph_data = {
                    'name': graph.name,
                    'type': 'nav',
                    'vertices': [],
                    'edges': []
                }
                for v in graph.vertices:
                    graph_data['vertices'].append({'x': v.x, 'y': v.y, 'name': v.name})
                
                for e in graph.edges:
                    graph_data['edges'].append({'v1': e.v1_idx, 'v2': e.v2_idx, 'type': e.edge_type})
                
                graphs.append(graph_data)

            # Process Wall Graph
            wall_graph = getattr(level, 'wall_graph', None)
            if wall_graph:
                graph_data = {
                    'name': wall_graph.name,
                    'type': 'wall',
                    'vertices': [],
                    'edges': []
                }
                for v in wall_graph.vertices:
                    graph_data['vertices'].append({'x': v.x, 'y': v.y, 'name': v.name})
                
                for e in wall_graph.edges:
                    graph_data['edges'].append({'v1': e.v1_idx, 'v2': e.v2_idx, 'type': e.edge_type})
                
                graphs.append(graph_data)
            
            level_data['graphs'] = graphs
            
            # Legacy fields for frontend compatibility (optional, can remove if frontend updated)
            level_data['vertices'] = [] 
            level_data['walls'] = []
            
            map_data['levels'].append(level_data)
            
        latest_building_map = map_data
        socketio.emit('map_update', map_data)

    def schedule_markers_callback(self, msg):
        global schedule_markers_cache
        
        # Process markers
        # Action: 0=ADD/MODIFY, 1=DEPRECATED, 2=DELETE, 3=DELETEALL
        
        for marker in msg.markers:
            ns = marker.ns
            mid = marker.id
            
            if marker.action == Marker.DELETEALL:
                schedule_markers_cache = {}
                continue
                
            if marker.action == Marker.DELETE:
                if ns in schedule_markers_cache and mid in schedule_markers_cache[ns]:
                    del schedule_markers_cache[ns][mid]
                continue
                
            if marker.action == Marker.ADD: # or MODIFY
                if ns not in schedule_markers_cache:
                    schedule_markers_cache[ns] = {}
                
                # Simplify data for frontend
                m_data = {
                    'type': marker.type, # 3=CYLINDER, 4=LINE_STRIP
                    'pose': {
                        'x': marker.pose.position.x,
                        'y': marker.pose.position.y
                    },
                    'scale': {
                        'x': marker.scale.x,
                        'y': marker.scale.y,
                        'z': marker.scale.z
                    },
                    'color': {
                        'r': marker.color.r,
                        'g': marker.color.g,
                        'b': marker.color.b,
                        'a': marker.color.a
                    },
                    'points': [],
                    'timestamp': time.time(),
                    'lifetime': marker.lifetime.sec + marker.lifetime.nanosec * 1e-9
                }
                
                for p in marker.points:
                    m_data['points'].append({'x': p.x, 'y': p.y})
                    
                schedule_markers_cache[ns][mid] = m_data

        # Emit updated cache
        # Note: Sending full cache might be heavy if there are many markers.
        # But for simple visualization it's okay.
        socketio.emit('schedule_markers_update', schedule_markers_cache)

# Flask Routes
@app.route('/')
def index():
    return render_template('index.html')

@app.route('/api/map')
def get_map():
    return json.dumps(latest_building_map) if latest_building_map else "{}"

@app.route('/api/fleets')
def get_fleets():
    return json.dumps(latest_fleet_states)

def main(args=None):
    rclpy.init(args=args)
    
    # Create the ROS node
    node = RmfWebVizNode()
    
    # Use MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    # Run ROS executor in a separate thread
    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()
    
    # Run Flask/SocketIO app
    print("Starting Web Server on http://0.0.0.0:5000")
    try:
        # allow_unsafe_werkzeug=True is needed for newer Flask-SocketIO versions when using threaded mode
        socketio.run(app, host='0.0.0.0', port=5000, debug=False, allow_unsafe_werkzeug=True)
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
