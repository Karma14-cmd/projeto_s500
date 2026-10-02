from pymavlink import mavutil
import curses
import time
import threading


# ==========================================
# CONFIGURAÇÕES
# ==========================================

CONEXAO = "udp:127.0.0.1:14550"

ALTURA_DECOLAGEM = 2.0

VELOCIDADE = 300

THRUST_NEUTRO = 500
THRUST_SUBIR = 650
THRUST_DESCER = 350


# ==========================================
# CONEXÃO
# ==========================================

print("==========================================")
print("       S500 - CONTROLE DE VOO")
print("==========================================")

print("\nConectando ao ArduPilot...")

veiculo = mavutil.mavlink_connection(CONEXAO)

print("Aguardando heartbeat...")

veiculo.wait_heartbeat()

print("\nHEARTBEAT RECEBIDO!")

print("Sistema:", veiculo.target_system)
print("Componente:", veiculo.target_component)


# ==========================================
# GUIDED
# ==========================================

print("\nMudando para GUIDED...")

veiculo.set_mode_apm("GUIDED")

time.sleep(2)

print("GUIDED configurado.")


# ==========================================
# ARM
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
# TAKEOFF
# ==========================================

print(f"\nDecolando para {ALTURA_DECOLAGEM} m...")

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
# AGUARDAR ALTURA
# ==========================================

while True:

    msg = veiculo.recv_match(
        type="GLOBAL_POSITION_INT",
        blocking=True
    )

    altitude = msg.relative_alt / 1000.0

    print(
        f"\rAltitude: {altitude:.2f} m",
        end=""
    )

    if altitude >= ALTURA_DECOLAGEM - 0.2:
        break


print("\n\nALTURA ATINGIDA!")

print("\nIniciando controle pelo teclado...")


# ==========================================
# ESTADO DO CONTROLE
# ==========================================

controle_ativo = True

pitch = 0
roll = 0
thrust = THRUST_NEUTRO
yaw = 0


# ==========================================
# ENVIO MAVLINK
# ==========================================

def enviar_controle():

    global controle_ativo

    while controle_ativo:

        veiculo.mav.manual_control_send(
            veiculo.target_system,
            pitch,
            roll,
            thrust,
            yaw,
            0
        )

        time.sleep(0.05)


# ==========================================
# MONITORAR ALTITUDE
# ==========================================

altitude_atual = ALTURA_DECOLAGEM


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
# CONTROLE DO TECLADO
# ==========================================

def controle_teclado(tela):

    global pitch
    global roll
    global thrust
    global yaw
    global controle_ativo

    curses.curs_set(0)

    tela.nodelay(True)

    tela.keypad(True)

    while controle_ativo:

        tecla = tela.getch()

        # ----------------------------------
        # NEUTRO A CADA CICLO
        # ----------------------------------

        pitch = 0
        roll = 0
        thrust = THRUST_NEUTRO
        yaw = 0


        # ----------------------------------
        # W - FRENTE
        # ----------------------------------

        if tecla in [ord("w"), ord("W")]:

            pitch = VELOCIDADE


        # ----------------------------------
        # S - TRÁS
        # ----------------------------------

        elif tecla in [ord("s"), ord("S")]:

            pitch = -VELOCIDADE


        # ----------------------------------
        # A - ESQUERDA
        # ----------------------------------

        elif tecla in [ord("a"), ord("A")]:

            roll = -VELOCIDADE


        # ----------------------------------
        # D - DIREITA
        # ----------------------------------

        elif tecla in [ord("d"), ord("D")]:

            roll = VELOCIDADE


        # ----------------------------------
        # SETA CIMA
        # ----------------------------------

        elif tecla == curses.KEY_UP:

            thrust = THRUST_SUBIR


        # ----------------------------------
        # SETA BAIXO
        # ----------------------------------

        elif tecla == curses.KEY_DOWN:

            thrust = THRUST_DESCER


        # ----------------------------------
        # SETA ESQUERDA
        # ----------------------------------

        elif tecla == curses.KEY_LEFT:

            yaw = -VELOCIDADE


        # ----------------------------------
        # SETA DIREITA
        # ----------------------------------

        elif tecla == curses.KEY_RIGHT:

            yaw = VELOCIDADE


        # ----------------------------------
        # Q - NEUTRO
        # ----------------------------------

        elif tecla in [ord("q"), ord("Q")]:

            pitch = 0
            roll = 0
            thrust = THRUST_NEUTRO
            yaw = 0


        # ----------------------------------
        # L - POUSO
        # ----------------------------------

        elif tecla in [ord("l"), ord("L")]:

            tela.clear()

            tela.addstr(
                0,
                0,
                "INICIANDO POUSO..."
            )

            tela.refresh()

            veiculo.set_mode_apm("LAND")

            controle_ativo = False

            break


        # ----------------------------------
        # TELA
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
            "        S500 - VOO MANUAL"
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
            f"Pitch : {pitch}"
        )

        tela.addstr(
            9,
            0,
            f"Roll  : {roll}"
        )

        tela.addstr(
            10,
            0,
            f"Thrust: {thrust}"
        )

        tela.addstr(
            11,
            0,
            f"Yaw   : {yaw}"
        )

        tela.addstr(
            13,
            0,
            "W/S  = Frente / Tras"
        )

        tela.addstr(
            14,
            0,
            "A/D  = Esquerda / Direita"
        )

        tela.addstr(
            15,
            0,
            "UP/DOWN = Subir / Descer"
        )

        tela.addstr(
            16,
            0,
            "LEFT/RIGHT = Girar"
        )

        tela.addstr(
            18,
            0,
            "Q = Neutro"
        )

        tela.addstr(
            19,
            0,
            "L = POUSAR"
        )

        tela.refresh()

        time.sleep(0.05)


# ==========================================
# THREAD MAVLINK
# ==========================================

thread_controle = threading.Thread(
    target=enviar_controle,
    daemon=True
)

thread_controle.start()


# ==========================================
# THREAD ALTITUDE
# ==========================================

thread_altitude = threading.Thread(
    target=monitorar_altitude,
    daemon=True
)

thread_altitude.start()


# ==========================================
# INICIAR TECLADO
# ==========================================

curses.wrapper(controle_teclado)


# ==========================================
# GARANTIR NEUTRO
# ==========================================

controle_ativo = False

veiculo.mav.manual_control_send(
    veiculo.target_system,
    0,
    0,
    THRUST_NEUTRO,
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


# ==========================================
# DESARMAR
# ==========================================

print("\n\nS500 pousou!")

time.sleep(2)

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
