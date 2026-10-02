from pymavlink import mavutil
import curses
import time
import threading

# ==========================================
# CONFIGURAÇÕES
# ==========================================

CONEXAO = "udp:127.0.0.1:14550"

ALTURA_DECOLAGEM = 2.0

VELOCIDADE = 1.0       # m/s
VELOCIDADE_YAW = 0.8   # rad/s

controle_ativo = True
altitude_atual = 0.0


# ==========================================
# CONEXÃO
# ==========================================

print("==========================================")
print("       S500 - CONTROLE GUIDED")
print("==========================================")

print("\nConectando ao ArduPilot...")

veiculo = mavutil.mavlink_connection(CONEXAO)

print("Aguardando heartbeat...")

veiculo.wait_heartbeat()

print("\nHEARTBEAT RECEBIDO!")

print("Sistema:", veiculo.target_system)
print("Componente:", veiculo.target_component)


# ==========================================
# MODO GUIDED
# ==========================================

print("\nMudando para GUIDED...")

veiculo.set_mode_apm("GUIDED")

time.sleep(2)

print("GUIDED configurado.")


# ==========================================
# ARMAR
# ==========================================

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


# ==========================================
# DECOLAGEM
# ==========================================

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


# ==========================================
# MONITORAR ALTITUDE
# ==========================================

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


print("\n\nALTURA DE 2 METROS ATINGIDA!")

time.sleep(2)

print("\nIniciando controle pelo teclado...")


# ==========================================
# FUNÇÃO PARA ENVIAR VELOCIDADE
# ==========================================

def enviar_velocidade(vx, vy, vz, yaw_rate):

    # BODY_NED:
    #
    # vx = frente
    # vy = direita
    # vz = baixo
    #
    # Portanto:
    #
    # vx positivo = frente
    # vx negativo = trás
    #
    # vy positivo = direita
    # vy negativo = esquerda
    #
    # vz positivo = descer
    # vz negativo = subir

    tipo = 1479

    veiculo.mav.set_position_target_local_ned_send(
        0,
        veiculo.target_system,
        veiculo.target_component,

        mavutil.mavlink.MAV_FRAME_BODY_NED,

        tipo,

        0, 0, 0,       # posição

        vx, vy, vz,    # velocidade

        0, 0, 0,       # aceleração

        0,             # yaw
        yaw_rate        # velocidade angular
    )


# ==========================================
# THREAD DE CONTROLE
# ==========================================

vx = 0.0
vy = 0.0
vz = 0.0
yaw_rate = 0.0


def enviar_controle():

    global controle_ativo

    while controle_ativo:

        enviar_velocidade(
            vx,
            vy,
            vz,
            yaw_rate
        )

        time.sleep(0.1)


# ==========================================
# MONITORAR ALTITUDE
# ==========================================

def monitorar_altitude():

    global altitude_atual
    global controle_ativo

    while controle_ativo:

        msg = veiculo.recv_match(
            type="GLOBAL_POSITION_INT",
            blocking=True,
            timeout=1
        )

        if msg:

            altitude_atual = msg.relative_alt / 1000.0


# ==========================================
# TECLADO
# ==========================================

def controle_teclado(tela):

    global vx
    global vy
    global vz
    global yaw_rate
    global controle_ativo

    curses.curs_set(0)

    tela.nodelay(True)
    tela.keypad(True)

    while controle_ativo:

        tecla = tela.getch()

        # ----------------------------------
        # PARAR MOVIMENTO
        # ----------------------------------

        vx = 0
        vy = 0
        vz = 0
        yaw_rate = 0

        # ----------------------------------
        # FRENTE
        # ----------------------------------

        if tecla in [ord("w"), ord("W")]:

            vx = VELOCIDADE

        # ----------------------------------
        # TRÁS
        # ----------------------------------

        elif tecla in [ord("s"), ord("S")]:

            vx = -VELOCIDADE

        # ----------------------------------
        # ESQUERDA
        # ----------------------------------

        elif tecla in [ord("a"), ord("A")]:

            vy = -VELOCIDADE

        # ----------------------------------
        # DIREITA
        # ----------------------------------

        elif tecla in [ord("d"), ord("D")]:

            vy = VELOCIDADE

        # ----------------------------------
        # SUBIR
        # ----------------------------------

        elif tecla == curses.KEY_UP:

            vz = -VELOCIDADE

        # ----------------------------------
        # DESCER
        # ----------------------------------

        elif tecla == curses.KEY_DOWN:

            vz = VELOCIDADE

        # ----------------------------------
        # GIRAR ESQUERDA
        # ----------------------------------

        elif tecla == curses.KEY_LEFT:

            yaw_rate = -VELOCIDADE_YAW

        # ----------------------------------
        # GIRAR DIREITA
        # ----------------------------------

        elif tecla == curses.KEY_RIGHT:

            yaw_rate = VELOCIDADE_YAW

        # ----------------------------------
        # PARAR
        # ----------------------------------

        elif tecla in [ord("q"), ord("Q")]:

            vx = 0
            vy = 0
            vz = 0
            yaw_rate = 0

        # ----------------------------------
        # POUSAR
        # ----------------------------------

        elif tecla in [ord("l"), ord("L")]:

            tela.clear()

            tela.addstr(
                0,
                0,
                "INICIANDO POUSO..."
            )

            tela.refresh()

            controle_ativo = False

            veiculo.set_mode_apm("LAND")

            break

        # ----------------------------------
        # INTERFACE
        # ----------------------------------

        tela.clear()

        tela.addstr(
            0,
            0,
            "=========================================="
        )

        tela.addstr(
            1,
            0,
            "        S500 - VOO GUIDED"
        )

        tela.addstr(
            2,
            0,
            "=========================================="
        )

        tela.addstr(
            4,
            0,
            f"Altitude: {altitude_atual:.2f} m"
        )

        tela.addstr(
            5,
            0,
            "Modo: GUIDED"
        )

        tela.addstr(
            6,
            0,
            "Estado: ARMADO"
        )

        tela.addstr(
            8,
            0,
            f"Velocidade X: {vx:.2f} m/s"
        )

        tela.addstr(
            9,
            0,
            f"Velocidade Y: {vy:.2f} m/s"
        )

        tela.addstr(
            10,
            0,
            f"Velocidade Z: {vz:.2f} m/s"
        )

        tela.addstr(
            11,
            0,
            f"Yaw: {yaw_rate:.2f} rad/s"
        )

        tela.addstr(
            13,
            0,
            "W = Frente"
        )

        tela.addstr(
            14,
            0,
            "S = Tras"
        )

        tela.addstr(
            15,
            0,
            "A = Esquerda"
        )

        tela.addstr(
            16,
            0,
            "D = Direita"
        )

        tela.addstr(
            17,
            0,
            "UP = Subir"
        )

        tela.addstr(
            18,
            0,
            "DOWN = Descer"
        )

        tela.addstr(
            19,
            0,
            "LEFT/RIGHT = Girar"
        )

        tela.addstr(
            21,
            0,
            "Q = Parar"
        )

        tela.addstr(
            22,
            0,
            "L = Pousar"
        )

        tela.refresh()

        time.sleep(0.05)


# ==========================================
# INICIAR THREADS
# ==========================================

thread_controle = threading.Thread(
    target=enviar_controle,
    daemon=True
)

thread_controle.start()


thread_altitude = threading.Thread(
    target=monitorar_altitude,
    daemon=True
)

thread_altitude.start()


# ==========================================
# INICIAR TECLADO
# ==========================================

curses.wrapper(
    controle_teclado
)


# ==========================================
# PARAR COMANDOS
# ==========================================

controle_ativo = False

enviar_velocidade(
    0,
    0,
    0,
    0
)


# ==========================================
# AGUARDAR POUSO
# ==========================================

print("\nAguardando pouso...")

while True:

    msg = veiculo.recv_match(
        type="GLOBAL_POSITION_INT",
        blocking=True
    )

    if msg:

        altitude = msg.relative_alt / 1000.0

        print(
            f"\rAltitude: {altitude:.2f} m",
            end=""
        )

        if altitude <= 0.3:
            break


print("\n\nS500 pousou!")

time.sleep(2)


# ==========================================
# DESARMAR
# ==========================================

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
