from mav import MAV2

dr = MAV2()


dr.takeoff(1)

HOVER_DISTANCE = 2
WAIT_FOR_DETECTION = 30
NUM_SHAKE = 4

dr.go_to_local(2, 2, 0)