# Use python threading to allow for tkinter and ros2 usage at the same time
import threading

# Use Tkinter for graphical interface
import tkinter as tk
from tkinter import *
from tkinter import ttk

# Use numpy for mathematical computations
import numpy as np

# Use matplotlib with tkinter module for live plots
import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure
import matplotlib.animation as animation
from matplotlib import style

# Use rclpy for interfacing with ROS2
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor

# Pegasus messages, actions and services for interfacing with the vehicle
from pegasus_msgs.msg import State
from pegasus_msgs.action import VehicleArm, VehicleLand


class MissionControlROS(Node):
    
    def __init__(self, namespace, id):
        # Initialize the mission control node for the specified drone 
        super().__init__(namespace + str(id) + '_mission_control')
        
        # Save the namespace and id of the drones
        self.namespace = namespace 
        self.id = id
        
        # Initialize the susbcribers, actions and publishers
        self.initialize_actions()
        self.initialize_subscribers()
        
        # State of the vehicle
        self.pos = np.zeros((3,))
        
    def initialize_actions(self):
        """
        Action clients to arm and land the vehicle
        """
        try:
            self.arm_client = ActionClient(self, VehicleArm, '/' + self.namespace + str(self.id) + '/arm')
            self.land_client = ActionClient(self, VehicleLand, '/' + self.namespace + str(self.id) + '/land')
        except Exception:
            print("Could not initialize the action clients for drone " + str(self.id))
        
    def initialize_subscribers(self):
        """
        Subscribers for the current state of the vehicle
        """
        self.state_sub = self.create_subscription(State, '/' + self.namespace + str(self.id) + '/nav/state', self.state_callback, 1)

    def state_callback(self, msg: State):
        """
        Callback to get the updated state of the vehicle
        """
        self.pos[0] = msg.pose.pose.position.x
        self.pos[1] = msg.pose.pose.position.y
        self.pos[2] = msg.pose.pose.position.z

class InitializeUI:
    
    def __init__(self, frame, mainUI, ros_executor):
        
        # The current frame where to place the widgets
        self.frame  = frame
        
        # The class that will define the main UI that will be instanced afterwards
        self.mainUI = mainUI
        self.ros_executor = ros_executor
        
        # UI variables
        self.entry_width = 10
        
        # Interest Variables
        self.namespace = "drone"
        self.number_vehicles = 1
        
        # Load the setup
        self.load_setup()
        
    def load_setup(self):
        
        # Define the number of drones
        tk.Label(self.frame, text="Number of Drones").grid(column=0, row=0)
        self.num_drones_entry = tk.Entry(self.frame, width=self.entry_width)
        self.num_drones_entry.grid(column=1, row=0)
        self.num_drones_entry.insert(0, str(self.number_vehicles))
        
        # Define the namespace of the drones
        tk.Label(self.frame, text="Namespace").grid(column=0, row=1)
        self.namespace_entry = tk.Entry(self.frame, width=self.entry_width)
        self.namespace_entry.grid(column=1, row=1)
        self.namespace_entry.insert(0, str(self.namespace))
        
        # Submit buttton
        tk.Button(self.frame, text="Set", width=self.entry_width, command=self.yield_control).grid(row=2, columnspan=2)
        
    def yield_control(self):
        
        # Read variables for the ROS2 namespace of the vehicles
        self.namespace = self.namespace_entry.get()
        
        # Read the variable for the number of the vehicles (currently the UI supports up to 10 vehicles)
        try:
            self.number_vehicles = int(self.num_drones_entry.get())
            if self.number_vehicles < 1 or self.number_vehicles > 10:
                self.number_vehicles = 1
                return
        except Exception:
            return
        
        # Clear the current frame
        for widgets in self.frame.winfo_children():
            widgets.destroy()
            
        # Prepare to instantiate the main UI
        self.mainUI(self.frame, self.namespace, self.number_vehicles, self.ros_executor)
        

class MainUI:
    
    def __init__(self, frame, namespace, num_vehicles, ros_executor):
        
        # Save the frame
        self.frame = frame
        
        # Interest Variables
        self.namespace = namespace
        self.number_vehicles = num_vehicles
        
        # ROS executor to process the data in parallel
        self.ros_executor: MultiThreadedExecutor = ros_executor
        
        # Initialize the ROS subscriptions and publishers to control the vehicles
        self.ros_vehicle_nodes = []
        thread = threading.Thread(target=self.load_vehicles)
        thread.start()
        
        # Load the UI
        self.plot_frame = None
        self.figure = Figure(figsize=(5,5), dpi=100)
        self.subplot = self.figure.add_subplot(111)
        self.load_control_buttons_ui()
        self.load_plots()
        self.ani = animation.FuncAnimation(self.figure, lambda i: self.animate(self.subplot, self.ros_vehicle_nodes, i), interval=1000, blit=False)
        
    def load_vehicles(self):
        
        # Create a node for each vehicle
        for i in range(self.number_vehicles):
            self.ros_vehicle_nodes.append(MissionControlROS(self.namespace, i + 1))
            self.ros_executor.add_node(self.ros_vehicle_nodes[-1])
            
        self.ros_executor.spin()

    def load_control_buttons_ui(self):
        
        # Load the controls for all vehicles
        for i in range(self.number_vehicles):
            
            # Inform the column to which drone it is associated
            tk.Label(self.frame, text=self.namespace + " " + str(i+1)).grid(column=0, row=i)
            
            # Arm/Disarm button
            ttk.Button(self.frame, text="Arm/Disarm", command=lambda:self.arm_callback(self.ros_vehicle_nodes[i])).grid(column=1, row=i)
            
            # Auto-land button
            ttk.Button(self.frame, text="Autoland", command=lambda:self.autoland_callback(self.ros_vehicle_nodes[i])).grid(column=2, row=i)
            
            # Kill Switch
            ttk.Button(self.frame, text="Kill Switch", command=lambda:self.killswitch_callback(self.ros_vehicle_nodes[i])).grid(column=3, row=i)
            
        # Make a new column for all the vehicles
        if self.number_vehicles != 1:
            tk.Label(self.frame, text="All vehicles")
            ttk.Button(self.frame, text="Arm/Disarm", command=self.arm_callback).grid(column=1, row=i+1)

    def load_plots(self):
        
        # Create a new sub-frame only for the canvas
        self.plot_frame = ttk.Frame(self.frame, padding=10)
        self.plot_frame.grid()
        
        canvas = FigureCanvasTkAgg(self.figure, self.plot_frame)
        canvas.get_tk_widget().grid(column=0, row=0)
        
        toolbar = NavigationToolbar2Tk(canvas, self.plot_frame, pack_toolbar=False)
        toolbar.update()
        
    @staticmethod
    def animate(plot, vehicle_nodes, i):
        
        # Plot XY position of each vehicle in the network
        for vehicle_node in vehicle_nodes:
            plot.scatter(vehicle_node.pos[0], vehicle_node.pos[1])

    @staticmethod
    def arm_callback(vehicle_node: MissionControlROS):
        
        # Create the arm action message
        arm_msg = VehicleArm.Goal()
        arm_msg.arm = True
        vehicle_node.arm_client.send_goal_async(arm_msg)
    
    @staticmethod
    def autoland_callback(vehicle_node: MissionControlROS):
        
        # Create the land action message
        land_msg = VehicleLand.Goal()
        land_msg.land = True
        vehicle_node.land_client.send_goal_async(land_msg)
    
    @staticmethod
    def killswitch_callback(vehicle_node: MissionControlROS):
        #TODO
        print("Killswitch not implemented yet")


def main(args=None):
    
    # Initilize the RCLPY library
    rclpy.init(args=args)
    
    # Create a ros executor to later process the callbacks without freezing the UI
    ros_executor = MultiThreadedExecutor(num_threads=1)
    
    # Create the main program window
    window = tk.Tk()
    window.title('Mission Control')
    
    # Create the frame inside the main window that will hold all the widgets, with a padding of 10 units
    frame = ttk.Frame(window, padding=10)
    frame.grid()
    
    # Load the first UI to define how many drone we will be controlling and let it take control of the system
    InitializeUI(frame, MainUI, ros_executor)
    
    # Run the window main loop
    window.mainloop()
    
if __name__ == "__main__":
    main()