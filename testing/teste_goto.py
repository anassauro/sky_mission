import sys
sys.path.append('/home/software/sky_ws/src/sky_mission/') # Adicionando o caminho para a pasta absoluto

from mav import MAV2 # Importando a função específica do arquivo 
import rospy
rospy.init_node('teste')

dr = MAV2()

dr.takeoff(1)

HOVER_DISTANCE = 2
WAIT_FOR_DETECTION = 30
NUM_SHAKE = 4

pos = (1, 1, 1)

dr.go_to_local(pos)
