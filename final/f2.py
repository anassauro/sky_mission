from mav import MAV2

dr = MAV2()

bases = ((-1, 1, 0), (-2, 2, 1), (-3, 2, 1.5), (1, 5, 0))

dr.takeoff(3)

HOVER_DISTANCE = 2
WAIT_FOR_DETECTION = 30
NUM_SHAKE = 4


for base in bases:

    try:
    
        dr.go_to_local((base[0], base[1], base[2] + HOVER_DISTANCE))

        for _ in range(NUM_SHAKE):
            dr.hold(WAIT_FOR_DETECTION/NUM_SHAKE)
            dr.shake()

    except KeyboardInterrupt:
        dr.go_to_local(0, 0, HOVER_DISTANCE)
        dr.land
        break

