from djitellopy import Tello
import time

# Conectar ao Tello
tello = Tello()
tello.connect()
#global pousou
pousou = False

bateria = tello.get_battery()
print("BATERIA:", bateria)

tello.takeoff()
time.sleep(3)
tello.move_forward(300)
tello.move_left(325)
tello.move_down(50)
#time.sleep(2)
altura_referencia = tello.get_distance_tof()

# Defina uma altura de referência
# Obtém a altura atual como referência

# Função para verificar a mudança na altura e aterrisar, se necessário
def verificar_mudanca_de_altura():
    altura_atual = tello.get_distance_tof()
    if abs(altura_atual - altura_referencia) > 15:  # Altere 10 para um valor adequado às suas necessidades
        global pousou
        tello.emergency()
        print("Base detectada abaixo! Pousando...")
        pousou = True


# Realize a verificação de mudança de altura
tempo_inicio = time.time()
duracao = 60                                                   #duração ciclo do 8
while not pousou:
    verificar_mudanca_de_altura()
    
    if ((time.time() - tempo_inicio) >= duracao):
        tello.takeoff()
        tello.move_right(325)
        tello.move_back(30)


#down 0.3



# Desconectar o Tello
tello.disconnect()