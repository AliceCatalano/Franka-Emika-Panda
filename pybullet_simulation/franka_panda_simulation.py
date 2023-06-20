import time
import pybullet as p
import pybullet_data
import socket 
import threading
from panda_robot import PandaRobot
import struct

INCLUDE_GRIPPER = True
DTYPE = 'float64'
SAMPLING_RATE = 1e-3  # 1000Hz sampling rate

data = [0.0167305000000000, -0.762614000000000, -0.0207622000000000, -2.34352000000000, -0.0305686000000000, 1.53975000000000, 0.753872000000000,
0.0167305000000000, -0.762614000000000, -0.0207622000000000, -2.34352000000000, -0.0305686000000000, 1.53975000000000, 0.753872000000000]

class myThread (threading.Thread):
   def __init__(self, threadID, name, counter):
      threading.Thread.__init__(self)
      self.threadID = threadID
      self.name = name
      self.counter = counter
   def run(self):
      print('Starting Thread')
      udpReceiver()
      print('Exiting Thread')

def udpReceiver():
    global data
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    client_socket.bind(("127.0.0.1", 1500))
    start = time.time()

    while(1):
        data, addr = client_socket.recvfrom(112) # double = 8 bytes * 7 joint
        data = struct.unpack('14d',data)


def main():
    """"""
    # Create two threads as follows
    thread1 = myThread(1, "Thread-1", 1)
    # Start new Threads
    thread1.start()
    # Basic Setup of environment
    physics_client_id = p.connect(p.GUI)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI,0) # disable pybullet GUI
    p.setTimeStep(SAMPLING_RATE)
    p.setGravity(0, 0, -9.81)

    # Setup plane
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    plane_id = p.loadURDF("plane.urdf")
    #p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,1)
    # Setup robot
    panda_robot = PandaRobot(False,include_gripper=INCLUDE_GRIPPER)#include_gripper=INCLUDE_GRIPPER
    panda_robot2 = PandaRobot(True,include_gripper=INCLUDE_GRIPPER)

    # Set up variables for simulation loop
    
    period = 1 / SAMPLING_RATE
    counter_seconds = -1

    # start simulation loop
    while(1):

        tmp = [data[0],data[1],data[2],data[3],data[4],data[5],data[6]] #necessary conversion to list
        tmp2 = [data[7],data[8],data[9],data[10],data[11],data[12],data[13]] #necessary conversion to list
        panda_robot.set_target_positions(tmp)
        panda_robot2.set_target_positions(tmp2)
        # Perform simulation step
        p.stepSimulation()
        time.sleep(SAMPLING_RATE)

    # Exit Simulation
    p.disconnect()
    print("Simulation end")


if __name__ == '__main__':
    main()
