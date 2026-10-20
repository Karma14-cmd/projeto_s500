from pymavlink import mavutil
import curses
import time


CONEXAO = "udp:127.0.0.1:14550"
ALTITUDE = 3


print("========================================")
print("       S500 - VOO PELO TECLADO")
print("========================================")

print("\nConectando...")

veiculo = mavutil.mavlink_connection(CONEXAO)

print("Aguardando heartbeat...")

veiculo.wait_heartbeat()

print("HEARTBEAT RECEBIDO!")

print("Sistema:", veiculo.target_system)
print("Componente:", veiculo.target_component)


# ========================================
# MUDAR PARA GUIDED
# ========================================

print("\nMudando para GUIDED...")

veiculo.set_mode_apm("GUIDED")

time.sleep(2)

print("Modo GUIDED configurado.")


# ========================================
# ARMAR
# ========================================

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


# ========================================
# DECOLAGEM
# ========================================

print(f"\nDecolando para {ALTITUDE} metros...")

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
    ALTITUDE
)


# ========================================
# AGUARDAR ALTITUDE
# ========================================

while True:

    mensagem = veiculo.recv_match(
        type="GLOBAL_POSITION_INT",
        blocking=True
    )

    altitude = mensagem.relative_alt / 1000.0

    print(
        f"\rAltitude: {altitude:.2f} m",
        end=""
    )

    if altitude >= ALTITUDE - 0.3:
        break


print("\n\nAltitude atingida!")

print("\nIniciando controle pelo teclado...")


# ========================================
# CONTROLE MAVLINK
# ========================================

def enviar(x, y, z, r):

    veiculo.mav.manual_control_send(
        veiculo.target_system,
        x,
        y,
        z,
        r,
        0
    )


# ========================================
# CONTROLE PELO TECLADO
# ========================================

def controle(tela):

    curses.curs_set(0)

    tela.nodelay(True)
    tela.keypad(True)

    x = 0
    y = 0
    z = 500
    r = 0

    while True:

        tecla = tela.getch()

        # -------------------------------
        # FRENTE
        # -------------------------------

        if tecla in [ord("w"), ord("W")]:
            x = 300

        # -------------------------------
        # TRÁS
        # -------------------------------

        elif tecla in [ord("s"), ord("S")]:
            x = -300

        # -------------------------------
        # ESQUERDA
        # -------------------------------

        elif tecla in [ord("a"), ord("A")]:
            y = -300

        # -------------------------------
        # DIREITA
        # -------------------------------

        elif tecla in [ord("d"), ord("D")]:
            y = 300

        # -------------------------------
        # SUBIR
        # -------------------------------

        elif tecla == curses.KEY_UP:
            z = 700

        # -------------------------------
        # DESCER
        # -------------------------------

        elif tecla == curses.KEY_DOWN:
            z = 300

        # -------------------------------
        # YAW ESQUERDA
        # -------------------------------

        elif tecla == curses.KEY_LEFT:
            r = -300

        # -------------------------------
        # YAW DIREITA
        # -------------------------------

        elif tecla == curses.KEY_RIGHT:
            r = 300

        # -------------------------------
        # Q = ESTABILIZAR
        # -------------------------------

        elif tecla in [ord("q"), ord("Q")]:

            x = 0
            y = 0
            z = 500
            r = 0

        # -------------------------------
        # L = POUSAR
        # -------------------------------

        elif tecla in [ord("l"), ord("L")]:

            print("\n\nPousando...")

            veiculo.set_mode_apm("LAND")

            return

        # -------------------------------
        # ENVIA CONTROLE
        # -------------------------------

        enviar(
            x,
            y,
            z,
            r
        )

        # -------------------------------
        # TELA
        # -------------------------------

        tela.clear()

        tela.addstr(
            0,
            0,
            "========================================"
        )

        tela.addstr(
            1,
            0,
            "        S500 - VOO MANUAL"
        )

        tela.addstr(
            2,
            0,
            "========================================"
        )

        tela.addstr(
            4,
            0,
            f"Pitch : {x}"
        )

        tela.addstr(
            5,
            0,
            f"Roll  : {y}"
        )

        tela.addstr(
            6,
            0,
            f"Thrust: {z}"
        )

        tela.addstr(
            7,
            0,
            f"Yaw   : {r}"
        )

        tela.addstr(
            9,
            0,
            "W/S  = Frente / Trás"
        )

        tela.addstr(
            10,
            0,
            "A/D  = Esquerda / Direita"
        )

        tela.addstr(
            11,
            0,
            "↑/↓  = Subir / Descer"
        )

        tela.addstr(
            12,
            0,
            "←/→  = Girar"
        )

        tela.addstr(
            14,
            0,
            "Q = Estabilizar"
        )

        tela.addstr(
            15,
            0,
            "L = Pousar"
        )

        tela.refresh()

        time.sleep(0.05)


# ========================================
# INICIAR CONTROLE
# ========================================

curses.wrapper(controle)


# ========================================
# AGUARDAR POUSO
# ========================================

print("Aguardando pouso...")

while True:

    mensagem = veiculo.recv_match(
        type="GLOBAL_POSITION_INT",
        blocking=True
    )

    altitude = mensagem.relative_alt / 1000.0

    print(
        f"\rAltitude: {altitude:.2f} m",
        end=""
    )

    if altitude <= 0.3:
        break


# ========================================
# DESARMAR
# ========================================

print("\n\nS500 pousou!")

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

print("\n========================================")
print("          MISSÃO FINALIZADA")
print("========================================")
