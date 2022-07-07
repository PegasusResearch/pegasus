#!/usr/bin/env python3
from pegasus_api import Drone
import time

def main(args=None):
     
    # Create the drone
    drone1 = Drone(id='6')
    
    # Arm the drone
    print('Arming the vehicle')
    drone1.arm()
    time.sleep(5)
    
    # Takeoff the drone
    print('Vehicle taking off')
    drone1.takeoff(height=1.0)
    
    # Sleep for 15 seconds
    time.sleep(15)     
    
    # Land the drone and disarm
    print('Vehicle Landing')
    drone1.land()
    drone1.disarm()
    
if __name__ == '__main__':
    main()