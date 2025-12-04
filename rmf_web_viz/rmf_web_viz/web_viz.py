import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rmf_fleet_msgs.msg import FleetState
from rmf_building_map_msgs.msg import BuildingMap
from rmf_site_map_msgs.msg import SiteMap
import threading
import flask
from flask import Flask, render_template
from flask_socketio import SocketIO
import json
import os

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
            level_data = {
                'name': level.name,
                'elevation': level.elevation,
                'images': []
            }
            
            for image in level.images:
                level_data['images'].append({
                    'name': image.name,
                    'x': image.x,
                    'y': image.y,
                    'scale': image.scale_x, # Assuming square pixels/scale
                    'width': image.width,
                    'height': image.height,
                    # Note: Image data itself (byte array) might be heavy to send repeatedly
                    # Ideally, serve it as a static file or blob on demand.
                    # For simplicity, we might skip sending raw data here and assume client requests it
                    # OR encode minimal info.
                    # RMF usually provides URLs or raw data. If raw data, we need to handle it.
                    'data_len': len(image.data)
                })
            
            # Extract walls/vertices for drawing if no image
            walls = []
            vertices = []
            
            for v in level.vertices:
                vertices.append({'x': v.x, 'y': v.y, 'name': v.name})
                
            for w in level.walls:
                walls.append({'v1': w.v1_idx, 'v2': w.v2_idx, 'type': w.wall_type})
                
            level_data['vertices'] = vertices
            level_data['walls'] = walls
            
            map_data['levels'].append(level_data)
            
        latest_building_map = map_data
        socketio.emit('map_update', map_data)

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
        socketio.run(app, host='0.0.0.0', port=5000, debug=False)
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
