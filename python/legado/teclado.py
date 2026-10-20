from pymavlink import mavutil
from pynput import keyboard
import time
import threading


# ==========================================
# CONFIGURAÇÃO
# ==========================================

CONEXAO = "udp:127.0.0.1:14550"

# Valores de controle
NEUTRO = 500
MAXIMO = 1000
MINIMO = 0

# Velocidade dos movimentos
MOVIMENTO = 300


# ==========================================
# CONEXÃO COM O S500
# ==========================================

print("======================================")
print("       CONTROLE DO S500 - TECLADO")
print("======================================")

print("\nConectando ao S500...")

veiculo = mavutil.mavlink_connection(CONEXAO)

print("Aguardando heartbeat...")

veiculo.wait_heartbeat()

print("\nHEARTBEAT RECEBIDO!")

print("Sistema:", veiculo.target_system)
print("Componente:", veiculo.target_component)

print("\nConexão estabelecida!")


# ==========================================
# ESTADO DAS TECLAS
# ==========================================

teclas = set()

x = 0       # Pitch
y = 0       # Roll
z = 500     # Throttle
r = 0       # Yaw


# ==========================================
# FUNÇÃO DE ENVIO
# ==========================================

def enviar_controle():

    global x, y, z, r

    while True:

        if "w" in teclas:
            x = MOVIMENTO

        elif "s" in teclas:
            x = -MOVIMENTO

        else:
            x = 0


        if "d" in teclas:
            y = MOVIMENTO

        elif "a" in teclas:
            y = -MOVIMENTO

        else:
            y = 0


        if "up" in teclas:
            z = 700

        elif "down" in teclas:
            z = 300

        else:
            z = 500


        if "right" in teclas:
            r = MOVIMENTO

        elif "left" in teclas:
            r = -MOVIMENTO

        else:
            r = 0


        # Envia MANUAL_CONTROL
        veiculo.mav.manual_control_send(
            veiculo.target_system,
            x,
            y,
            z,
            r,
            0
        )

        time.sleep(0.1)


# ==========================================
# TECLA PRESSIONADA
# ==========================================

def tecla_pressionada(tecla):

    try:

        if tecla.char in ["w", "a", "s", "d"]:
            teclas.add(tecla.char)

    except AttributeError:

        if tecla == keyboard.Key.up:
            teclas.add("up")

        elif tecla == keyboard.Key.down:
            teclas.add("down")

        elif tecla == keyboard.Key.left:
            teclas.add("left")

        elif tecla == keyboard.Key.right:
            teclas.add("right")


# ==========================================
# TECLA LIBERADA
# ==========================================

def tecla_liberada(tecla):

    try:

        if tecla.char in ["w", "a", "s", "d"]:
            teclas.discard(tecla.char)

    except AttributeError:

        if tecla == keyboard.Key.up:
            teclas.discard("up")

        elif tecla == keyboard.Key.down:
            teclas.discard("down")

        elif tecla == keyboard.Key.left:
            teclas.discard("left")

        elif tecla == keyboard.Key.right:
            teclas.discard("right")

    # Q = parar
    if hasattr(tecla, "char") and tecla.char == "q":

        teclas.clear()

        print("\nMovimentos zerados.")


    # X = sair
    if hasattr(tecla, "char") and tecla.char == "x":

        print("\nEncerrando controle...")

        teclas.clear()

        return False


# ==========================================
# THREAD DE CONTROLE
# ==========================================

thread = threading.Thread(
    target=enviar_controle,
    daemon=True
)

thread.start()


# ==========================================
# INFORMAÇÕES
# ==========================================

print("""
========================================

CONTROLES

W       Frente
S       Trás

A       Esquerda
D       Direita

↑       Subir
↓       Descer

←       Girar esquerda
→       Girar direita

Q       Parar

X       Sair

========================================
""")

print("Controle ativo!")
print("Pressione X para sair.")


# ==========================================
# MONITORAMENTO DO TECLADO
# ==========================================

with keyboard.Listener(
    on_press=tecla_pressionada,
    on_release=tecla_liberada
) as listener:

    listener.join()


print("\nPrograma finalizado.")
