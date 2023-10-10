from mav import MAV2

dr = MAV2()

base_alta = ((-4.95, 0.9, -0.5))

HOVER_DISTANCE = 2.5
WAIT_FOR_DETECTION = 30
NUM_SHAKE = 4

dr.takeoff(HOVER_DISTANCE)


#for base in bases:

try:
    dr.go_to_local((base_alta[0], base_alta[1], HOVER_DISTANCE))
    for _ in range(NUM_SHAKE):
        dr.hold(WAIT_FOR_DETECTION/NUM_SHAKE)
        dr.shake()
    
except KeyboardInterrupt:
    dr.go_to_local(0, 0, HOVER_DISTANCE)
    dr.land

dr.go_to_local(0, 0, HOVER_DISTANCE)
dr.land



