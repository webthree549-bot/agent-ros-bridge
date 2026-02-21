"""
Builder Node - Real ROS2 Node
Constructs structures and repairs equipment
Publishes to /mars/telemetry, /mars/mission
Subscribes to /mars/commands
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import random


class BuilderNode(Node):
    """ROS2 Node for construction robot"""
    
    def __init__(self, robot_id="builder_1"):
        super().__init__(f'builder_{robot_id}')
        self.robot_id = robot_id
        
        # Builder state
        self.position = {'x': random.uniform(-10, 10), 'y': random.uniform(-10, 10)}
        self.battery = 100.0
        self.materials = {'steel': 100, 'concrete': 100, 'glass': 50, 'electronics': 30}
        self.status = 'idle'
        self.construction_speed = random.uniform(0.8, 1.5)
        
        # Projects
        self.current_project = None
        self.projects_completed = []
        self.build_queue = []
        
        # Publishers
        self.telemetry_pub = self.create_publisher(String, '/mars/telemetry', 10)
        self.mission_pub = self.create_publisher(String, '/mars/mission', 10)
        
        # Subscribers
        self.command_sub = self.create_subscription(
            String, '/mars/commands', self.command_callback, 10
        )
        
        # Timers
        self.telemetry_timer = self.create_timer(2.0, self.publish_telemetry)
        self.work_timer = self.create_timer(3.0, self.do_construction)
        self.battery_timer = self.create_timer(10.0, self.drain_battery)
        
        self.get_logger().info(f'🏗️  Builder {self.robot_id} ready for construction')
        
    def do_construction(self):
        """Main construction work"""
        if self.battery < 15:
            self.status = 'low_battery'
            return
            
        if self.status == 'building' and self.current_project:
            project = self.current_project
            progress_increment = random.uniform(2, 5) * self.construction_speed
            project['progress'] = min(100, project['progress'] + progress_increment)
            
            # Consume materials
            for material, amount in project['materials_needed'].items():
                consumed = amount * (progress_increment / 100)
                self.materials[material] = max(0, self.materials[material] - consumed)
                
            if project['progress'] >= 100:
                self.complete_project()
                
    def complete_project(self):
        """Complete current project"""
        if self.current_project:
            project = self.current_project
            project['completed'] = True
            project['completion_time'] = self.get_clock().now().to_msg().__str__()
            self.projects_completed.append(project)
            
            self.get_logger().info(f'✅ Completed: {project["name"]}')
            
            # Publish mission update
            mission_msg = {
                'type': 'construction_complete',
                'robot_id': self.robot_id,
                'project': project,
                'timestamp': self.get_clock().now().to_msg().__str__()
            }
            msg = String()
            msg.data = json.dumps(mission_msg)
            self.mission_pub.publish(msg)
            
            self.current_project = None
            self.status = 'idle'
            
            # Start next project if available
            if self.build_queue:
                self.start_project(self.build_queue.pop(0))
                
    def start_project(self, project):
        """Start a new construction project"""
        # Check materials
        for material, amount in project['materials_needed'].items():
            if self.materials.get(material, 0) < amount:
                self.get_logger().warn(f'⚠️  Insufficient {material} for {project["name"]}')
                return False
                
        self.current_project = {
            'name': project['name'],
            'type': project['type'],
            'progress': 0.0,
            'materials_needed': project['materials_needed'].copy(),
            'position': project.get('position', self.position.copy()),
            'completed': False
        }
        self.status = 'building'
        self.get_logger().info(f'🏗️  Starting construction: {project["name"]}')
        return True
        
    def publish_telemetry(self):
        """Publish telemetry data"""
        telemetry = {
            'robot_id': self.robot_id,
            'type': 'builder',
            'position': self.position.copy(),
            'battery': round(self.battery, 1),
            'status': self.status,
            'materials': {k: round(v, 1) for k, v in self.materials.items()},
            'current_project': self.current_project,
            'projects_completed': len(self.projects_completed),
            'queue_length': len(self.build_queue),
            'timestamp': self.get_clock().now().to_msg().__str__()
        }
        
        msg = String()
        msg.data = json.dumps(telemetry)
        self.telemetry_pub.publish(msg)
        
    def drain_battery(self):
        """Simulate battery drain"""
        drain = 1.5
        if self.status == 'building':
            drain = 4.0
        self.battery = max(0, self.battery - drain)
        
    def command_callback(self, msg):
        """Handle commands"""
        try:
            cmd = json.loads(msg.data)
            if cmd.get('target') == self.robot_id or cmd.get('target') == 'all_builders':
                action = cmd.get('action')
                
                if action == 'build':
                    project = {
                        'name': cmd.get('project_name', 'Unnamed Structure'),
                        'type': cmd.get('project_type', 'habitat'),
                        'materials_needed': cmd.get('materials', {'steel': 20, 'concrete': 30}),
                        'position': cmd.get('position', self.position.copy())
                    }
                    if self.status == 'idle':
                        self.start_project(project)
                    else:
                        self.build_queue.append(project)
                        self.get_logger().info(f'📋 Added {project["name"]} to queue')
                        
                elif action == 'stop':
                    self.status = 'idle'
                    self.get_logger().info('🛑 Construction paused')
                    
                elif action == 'resupply':
                    for material in self.materials:
                        self.materials[material] = min(100, self.materials[material] + 30)
                    self.get_logger().info('📦 Materials restocked')
                    
        except json.JSONDecodeError:
            pass


def main(args=None):
    import os
    rclpy.init(args=args)
    
    robot_id = os.environ.get('ROBOT_ID', 'builder_1')
    node = BuilderNode(robot_id=robot_id)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
