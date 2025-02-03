import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from pegasus_msgs.srv import Waypoint, AddCircle, SetMode
from pegasus_msgs.msg import AutopilotStatus
from nav_msgs.msg import Odometry


class Drone(Node):

    def __init__(self, id):
        super().__init__('drone_api_' + str(id))

        self.id = id
        self.namespace = 'drone'

        self.set_autopilot_srv = self.create_client(SetMode, '/drone' + str(id) +'/autopilot/change_mode')
        print('Initializing service: /drone' + str(id) +'/autopilot/change_mode')
        while not self.set_autopilot_srv.wait_for_service(timeout_sec=10.0):
            self.get_logger().info('service not available, waiting again...')

        # Create subscriptions
        self.create_subscription(AutopilotStatus, '/drone' + str(id) + '/autopilot/status', self.autopilot_status_callback, qos_profile_sensor_data)

        # Create subscriptions to listen to the drone's position (replace PositionStatus with your message type)
        self.create_subscription(Odometry, '/drone' + str(id) + '/fmu/filter/state', self.position_status_callback, qos_profile_sensor_data)

        # Request msgs
        self.set_mode_req = SetMode.Request()

    def autopilot_status_callback(self, msg):
        self.get_logger().info('Received autopilot status: %s' % msg.mode)

    def set_autopilot_mode(self, mode='DisarmMode'):

        self.get_logger().info('Setting autopilot mode to: %s' % mode)
            
        # Set the mode request
        self.set_mode_req.mode = mode
        
        # Make an async request
        self.future = self.set_autopilot_srv.call_async(self.set_mode_req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    
    def position_status_callback(self, msg):
        self.pos_x = msg.pose.pose.position.x
        self.pos_y = msg.pose.pose.position.y
        self.pos_z = msg.pose.pose.position.z
           

def main(args=None):
    rclpy.init(args=args)

    drones = []
    n_drones = 4
    first_drone_id = 7

    for i in range(n_drones):
        drones.append(Drone(i+first_drone_id))

    for i in range(n_drones):
        drones[i].set_autopilot_mode('LandMode')

    # Shutdown the demo
    for i in range(n_drones):
        drones[i].destroy_node()

    rclpy.shutdown()

if __name__ == "__main__":
    main()
