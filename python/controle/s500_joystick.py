from pymavlink import mavutil
import curses
import time
import threading
import math

# ============================================================
# CONFIGURAÇÕES
# ============================================================

CONEXAO = "udp:127.0.0.1:14550"

ALTURA_DECOLAGEM = 2.0

# Velocidades máximas
MAX_VX = 2.0       # frente/trás - m/s
MAX_VY = 2.0       # esquerda/direita - m/s
MAX_VZ = 1.0       # subida/descida - m/s
MAX_YAW = 1.2      # giro - rad/s

# Aceleração
ACELERACAO = 0.08

# Desaceleração
DESACELERACAO = 0.12

# Frequência do comando
PERIODO = 0.05

controle_ativo = True
altitude_atual = 0.0

# ============================================================
# ESTADOS DO JOYSTICK
# ============================================================

vx = 0.0
vy = 0.0
vz = 0.0
yaw = 0.0

teclas = set()


# ============================================================
# CONEXÃO
# ============================================================

print("==========================================")
print("       S500 - JOYSTICK VIRTUAL")
print("==========================================")

print("\nConectando ao ArduPilot...")

veiculo = mavutil.mavlink_connection(CONEXAO)

print("Aguardando heartbeat...")

veiculo.wait_heartbeat()

print("\nHEARTBEAT RECEBIDO!")

print("Sistema:", veiculo.target_system)
print("Componente:", veiculo.target_component)


# ============================================================
# GUIDED
# ============================================================

print("\nMudando para GUIDED...")

veiculo.set_mode_apm("GUIDED")

time.sleep(2)

print("GUIDED configurado.")


# ============================================================
# ARM
# ============================================================

print("\nArmando S500...")

veiculo.mav.command_long_send(
    veiculo.target_system,
    veiculo.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1,
    0,
    0,
    0,
    0,
    0,
    0
)

veiculo.motors_armed_wait()

print("S500 ARMADO!")


# ============================================================
# TAKEOFF
# ============================================================

print(f"\nDecolando para {ALTURA_DECOLAGEM} metros...")

veiculo.mav.command_long_send(
    veiculo.target_system,
    veiculo.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    ALTURA_DECOLAGEM
)


# ============================================================
# ESPERAR 2 METROS
# ============================================================

while True:

    msg = veiculo.recv_match(
        type="GLOBAL_POSITION_INT",
        blocking=True
    )

    if msg:

        altitude_atual = msg.relative_alt / 1000.0

        print(
            f"\rAltitude: {altitude_atual:.2f} m",
            end=""
        )

        if altitude_atual >= ALTURA_DECOLAGEM - 0.2:
            break


print("\n\nALTURA ATINGIDA!")

time.sleep(2)


# ============================================================
# COMANDO GUIDED
# ============================================================

def enviar_velocidade():

    # BODY_NED
    #
    # X positivo = frente
    # X negativo = trás
    #
    # Y positivo = direita
    # Y negativo = esquerda
    #
    # Z positivo = descer
    # Z negativo = subir

    tipo_mascara = 1479

    veiculo.mav.set_position_target_local_ned_send(

        0,

        veiculo.target_system,
        veiculo.target_component,

        mavutil.mavlink.MAV_FRAME_BODY_NED,

        tipo_mascara,

        0, 0, 0,

        vx,
        vy,
        vz,

        0, 0, 0,

        0,
        yaw
    )


# ============================================================
# FUNÇÃO DE ACELERAÇÃO
# ============================================================

def aproximar(valor, alvo, passo):

    if valor < alvo:

        valor += passo

        if valor > alvo:
            valor = alvo

    elif valor > alvo:

        valor -= passo

        if valor < alvo:
            valor = alvo

    return valor


# ============================================================
# CONTROLE DO JOYSTICK
# ============================================================

def atualizar_joystick():

    global vx
    global vy
    global vz
    global yaw

    # --------------------------------------------
    # ALVOS
    # --------------------------------------------

    alvo_vx = 0.0
    alvo_vy = 0.0
    alvo_vz = 0.0
    alvo_yaw = 0.0

    # --------------------------------------------
    # FRENTE / TRÁS
    # --------------------------------------------

    if "w" in teclas:

        alvo_vx = MAX_VX

    elif "s" in teclas:

        alvo_vx = -MAX_VX

    # --------------------------------------------
    # ESQUERDA / DIREITA
    # --------------------------------------------

    if "a" in teclas:

        alvo_vy = -MAX_VY

    elif "d" in teclas:

        alvo_vy = MAX_VY

    # --------------------------------------------
    # SUBIR / DESCER
    # --------------------------------------------

    if "up" in teclas:

        alvo_vz = -MAX_VZ

    elif "down" in teclas:

        alvo_vz = MAX_VZ

    # --------------------------------------------
    # YAW
    # --------------------------------------------

    if "left" in teclas:

        alvo_yaw = -MAX_YAW

    elif "right" in teclas:

        alvo_yaw = MAX_YAW

    # --------------------------------------------
    # ACELERAÇÃO
    # --------------------------------------------

    vx = aproximar(
        vx,
        alvo_vx,
        ACELERACAO
    )

    vy = aproximar(
        vy,
        alvo_vy,
        ACELERACAO
    )

    vz = aproximar(
        vz,
        alvo_vz,
        ACELERACAO
    )

    yaw = aproximar(
        yaw,
        alvo_yaw,
        ACELERACAO
    )


# ============================================================
# THREAD PRINCIPAL DO JOYSTICK
# ============================================================

def thread_joystick():

    global controle_ativo

    while controle_ativo:

        atualizar_joystick()

        enviar_velocidade()

        time.sleep(PERIODO)


# ============================================================
# MONITORAR ALTITUDE
# ============================================================

def monitorar_altitude():

    global altitude_atual

    while controle_ativo:

        msg = veiculo.recv_match(
            type="GLOBAL_POSITION_INT",
            blocking=True,
            timeout=1
        )

        if msg:

            altitude_atual = (
                msg.relative_alt / 1000.0
            )


# ============================================================
# TECLADO
# ============================================================

def teclado(tela):

    global controle_ativo
    global vx
    global vy
    global vz
    global yaw

    curses.curs_set(0)

    tela.nodelay(True)

    tela.keypad(True)

    while controle_ativo:

        tecla = tela.getch()

        # -----------------------------------------
        # TECLAS
        # -----------------------------------------

        if tecla == ord("w") or tecla == ord("W"):

            teclas.add("w")

        elif tecla == ord("s") or tecla == ord("S"):

            teclas.add("s")

        elif tecla == ord("a") or tecla == ord("A"):

            teclas.add("a")

        elif tecla == ord("d") or tecla == ord("D"):

            teclas.add("d")

        elif tecla == curses.KEY_UP:

            teclas.add("up")

        elif tecla == curses.KEY_DOWN:

            teclas.add("down")

        elif tecla == curses.KEY_LEFT:

            teclas.add("left")

        elif tecla == curses.KEY_RIGHT:

            teclas.add("right")

        # -----------------------------------------
        # PARAR
        # -----------------------------------------

        elif tecla == ord("q") or tecla == ord("Q"):

            teclas.clear()

            vx = 0
            vy = 0
            vz = 0
            yaw = 0

        # -----------------------------------------
        # POUSAR
        # -----------------------------------------

        elif tecla == ord("l") or tecla == ord("L"):

            controle_ativo = False

            teclas.clear()

            veiculo.set_mode_apm("LAND")

            break

        # -----------------------------------------
        # TELA
        # -----------------------------------------

        tela.clear()

        tela.addstr(
            0,
            0,
            "=========================================="
        )

        tela.addstr(
            1,
            0,
            "        S500 - JOYSTICK VIRTUAL"
        )

        tela.addstr(
            2,
            0,
            "=========================================="
        )

        tela.addstr(
            4,
            0,
            f"Altitude : {altitude_atual:.2f} m"
        )

        tela.addstr(
            5,
            0,
            "Modo     : GUIDED"
        )

        tela.addstr(
            7,
            0,
            f"VX : {vx:6.2f} m/s"
        )

        tela.addstr(
            8,
            0,
            f"VY : {vy:6.2f} m/s"
        )

        tela.addstr(
            9,
            0,
            f"VZ : {vz:6.2f} m/s"
        )

        tela.addstr(
            10,
            0,
            f"YAW: {yaw:6.2f} rad/s"
        )

        tela.addstr(
            12,
            0,
            "W/S       Frente / Tras"
        )

        tela.addstr(
            13,
            0,
            "A/D       Esquerda / Direita"
        )

        tela.addstr(
            14,
            0,
            "UP/DOWN   Subir / Descer"
        )

        tela.addstr(
            15,
            0,
            "LEFT/RIGHT Girar"
        )

        tela.addstr(
            17,
            0,
            "Q = PARADA"
        )

        tela.addstr(
            18,
            0,
            "L = POUSAR"
        )

        tela.addstr(
            20,
            0,
            "Controle proporcional"
        )

        tela.refresh()

        time.sleep(0.03)


# ============================================================
# THREADS
# ============================================================

thread1 = threading.Thread(
    target=thread_joystick,
    daemon=True
)

thread1.start()


thread2 = threading.Thread(
    target=monitorar_altitude,
    daemon=True
)

thread2.start()


# ============================================================
# INICIAR TECLADO
# ============================================================

print("\nControle iniciado!")

curses.wrapper(teclado)


# ============================================================
# FINALIZAR CONTROLE
# ============================================================

controle_ativo = False

teclas.clear()

time.sleep(1)


# ============================================================
# AGUARDAR POUSO
# ============================================================

print("\nAguardando pouso...")

while True:

    msg = veiculo.recv_match(
        type="GLOBAL_POSITION_INT",
        blocking=True
    )

    if msg:

        altitude = (
            msg.relative_alt / 1000.0
        )

        print(
            f"\rAltitude: {altitude:.2f} m",
            end=""
        )

        if altitude <= 0.3:

            break


print("\n\nS500 pousou!")

time.sleep(2)


# ============================================================
# DESARMAR
# ============================================================

print("Desarmando...")

veiculo.mav.command_long_send(
    veiculo.target_system,
    veiculo.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0
)


print("\n==========================================")
print("          MISSÃO FINALIZADA")
print("==========================================")
