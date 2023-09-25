# Importar a biblioteca DroneKit
from dronekit import connect, VehicleMode
import time
import math
from pymavlink import mavutil
import rospy
from std_msgs.msg import String

# Conectar ao drone através de uma porta serial ou rede (por exemplo, UDP)
connection_string = 'tcp:127.0.0.1:5763'
vehicle = connect(connection_string)

class BarcodeAnalyzer():
    
    def __init__(self) -> None:
        self.barcode = String()
        self.barcode_list = []
        rospy.init_node('sky_vision_barcode_analyzer', anonymous=False)
        
        rospy.Subscriber('/sky_vision/down_cam/barcode_read', String, self.callback)
    
    def callback(self, message):
        self.barcode = message.data
        if self.barcode not in self.barcode_list:
            self.barcode_list.append(self.barcode)

# Definir a função para imprimir informações de telemetria
def imprimir_telemetria():
    
    print("\nTELEMETRIA")

    # Imprimir informações de posição GPS
    print("Posição GPS: Lat = {0}, Lon = {1}, Alt = {2}".format(
        vehicle.location.global_frame.lat,
        vehicle.location.global_frame.lon,
        vehicle.location.global_frame.alt
    ))

    # Imprimir informações de atitude
    print("Atitude: Roll = {0}, Pitch = {1}, Yaw = {2}".format(
        vehicle.attitude.roll,
        vehicle.attitude.pitch,
        vehicle.attitude.yaw
    ))

    # Imprimir informações de velocidade
    print("Velocidade: Vx = {0}, Vy = {1}, Vz = {2}".format(
        vehicle.velocity[0],
        vehicle.velocity[1],
        vehicle.velocity[2]
    ))

    # Imprimir altura relativa do drone
    print(f"Altitude relativa (m): {vehicle.location.global_relative_frame.alt}\n")

    # Aguardar um curto intervalo de tempo antes de imprimir novamente
    time.sleep(1)

def arm_and_takeoff(altitude):
    print("Armando os motores...")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.is_armable:
        print("Esperando para armar...")
        time.sleep(1)

    while not vehicle.armed:
        print("Esperando para armar...")
        time.sleep(1)

    print("Decolando...")
    vehicle.simple_takeoff(altitude)

    while True:
        if vehicle.location.global_relative_frame.alt >= altitude * 0.95:
            print("Altitude de ", altitude, " metros alcançada.")
            break
        time.sleep(1)

def land():
    print("Iniciando o procedimento de pouso...")
    vehicle.mode = VehicleMode("LAND")

    while not vehicle.mode.name == "LAND":
        time.sleep(1)

    print("Pousando...")
    while vehicle.location.global_relative_frame.alt > 0.1:
        time.sleep(1)

    print("Drone pousou com sucesso.")

# Função para retornar e pousar
def retornar_e_pousar():
    vehicle.mode = VehicleMode("RTL")  # Definir o modo de voo como "Return to Launch"
    print("Retornando e pousando...")

def print_status():
    #-- Check vehicle status
    print("\nSTATUS")
    print(f"Mode: {vehicle.mode.name}")
    print(" Global Location: %s" % vehicle.location.global_frame)
    print(" Global Location (relative altitude): %s" % vehicle.location.global_relative_frame)
    print(" Local Location: %s" % vehicle.location.local_frame)
    print(" Gimbal status: %s" % vehicle.gimbal)
    print(" EKF OK?: %s" % vehicle.ekf_ok)
    print(" Last Heartbeat: %s" % vehicle.last_heartbeat)
    print(" Is Armable?: %s" % vehicle.is_armable)
    print(" System status: %s" % vehicle.system_status.state)
    print(" Armed: %s" % vehicle.armed)    # settable

def goto_position_target_local_ned(north, east, down, yaw=0):
    """
    Send SET_POSITION_TARGET_LOCAL_NED command to request the vehicle fly to a specified
    location in the North, East, Down frame.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        000000000000000000, # type_mask (only positions enabled)
        north, east, down,
        0, 0, 0, # x, y, z velocity in m/s  (not used)
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        yaw, 0)    # yaw, yaw_rate
    
    # send command to vehicle
    vehicle.send_mavlink(msg)

def condition_yaw(heading, relative=False):
    if relative:
        is_relative=1 #yaw relative to direction of travel
    else:
        is_relative=0 #yaw is an absolute angle
    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        heading,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        1,          # param 3, direction -1 ccw, 1 cw
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)

# Função para mover o drone para uma posição específica mantendo o yaw constante
def move_to_target(east, altitude):
    condition_yaw(0, relative=True)  # Manter o yaw constante
    goto_position_target_local_ned(0, east, -altitude)  # Use altitude negativa para subir

    # Aguarde até que o drone alcance a posição desejada
    while True:
        current_position = vehicle.location.local_frame
        distance_to_target = abs(east - current_position.east)
        if distance_to_target < 0.5:  # Defina uma tolerância para considerar que o drone atingiu a posição
            break
        time.sleep(1)

# Programa principal
try:
    ba = BarcodeAnalyzer()
    imprimir_telemetria()
    arm_and_takeoff(1)  # Decolar para 1 metro de altitude
    rospy.spin()

    for i in range(4):  # Realizar os movimentos de ida e volta 4 vezes
        for j in range(5):  # Mover 1 metro para a direita e parar por 1 segundo, repetir 5 vezes
            print("Movendo para a direita...")
            move_to_target(1 + j, 1 + (0.5*i))  # Mover 1 metro para a direita

            print("Parando por 1 segundo...")
            time.sleep(1)  # Parar por 1 segundo

        print("Movendo para a esquerda...")
        move_to_target(0, 1 + (0.5*i))  # Mover 5 metros para a esquerda

        print("Aguardando por 2 segundos...")
        time.sleep(2)  # Aguardar por 2 segundos

    time.sleep(5)
    print("Pousando...")
    retornar_e_pousar()  # Pousar o drone

except KeyboardInterrupt:
    print("Aplicativo interrompido pelo usuário.")

finally:
    # Desarmar os motores e fechar a conexão com o drone
    vehicle.armed = False
    vehicle.close()
    print(ba.barcode_list)