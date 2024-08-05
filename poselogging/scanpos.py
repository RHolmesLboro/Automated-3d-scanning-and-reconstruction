import roslibpy, csv, time
from robot_client import RobotClient
from scipy.spatial.transform import rotation as R

try:
  client = roslibpy.Ros(host='192.168.3.100', port=9090) # Change host to the IP of the robot
  client.run()
except:
  print("can't connect to the robot, check your IP address and network connection")
  exit()

# Sanity check to see if we are connected
print('Verifying the ROS target is connected?', client.is_connected)

rc = RobotClient(client)

time.sleep(1) # To give it time to connect

if rc.get_arm_power() == False: # If the arm is off turn it on
  rc.arm_power_on()
while(rc.get_arm_active() == False): # Wait for the arm to be active
  print("Initialising Arm..")
  time.sleep(1)

rc.robot_arm_enable()
time.sleep(1)

f = open("TCP.csv", "w+")

while rc.get_arm_power():
    i = input("Press enter to log coords or q to quit:")
    if i == "Q" or i == "q":
      rc.robot_arm_disable()
      break
    else:
      coords = rc.get_tcp_coordinates()
      rot = rc.get_tcp_orientation()
      rot = R.from_quat([rot["x"], rot["y"], rot["z"], rot["w"]], scalar_first = False).as_euler()
      print(coords, "\n", rot)
      pos = [coords["x"], coords["y"], coords["z"], rot["x"], rot["y"], rot["z"]]
      csv.writer(f, delimiter=" ", quotechar="¦", quoting=csv.QUOTE_MINIMAL).writerow(pos)
