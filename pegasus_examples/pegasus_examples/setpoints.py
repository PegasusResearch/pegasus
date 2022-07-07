#!/usr/bin/env python3
from pegasus_api import Drone
import time

def main(args=None):
     
    # Create the drone
    drone1 = Drone(id='1')
    
    # Arm the drone
    print('Arming the vehicle')
    drone1.arm()
    time.sleep(5)
    
    # Get the current time
    initial_time = time.time()
    curr_time = 0
    
    try:
        print('Vehicle setting desired position')
        # Control the drone at approximately 20 Hz
        while(curr_time < 15.0):
            
            # Set a waypoint for the vehicle to follow using the internal position controller
            drone1.set_position(x=0.0, y=0.0, z=-2.5, yaw=90.0)
        
            # Update the current elapsed time  
            curr_time = time.time() - initial_time
            
            time.sleep(0.05)  
        
        # Land the drone and disarm
        print('Vehicle Landing')
        drone1.land()
        drone1.disarm()
    
    except KeyboardInterrupt:
        pass
    
    drone1.shutdown()
    
if __name__ == '__main__':
    main()