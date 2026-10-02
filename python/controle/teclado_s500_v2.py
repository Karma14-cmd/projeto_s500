from pymavlink import mavutil
import curses
import time


# ==========================================
# CONFIGURAÇÃO
# ==========================================

CONEXAO = "udp:127.0.0.1:14550"

VELOCIDADE = 300

NEUTRO = 500


# ==========================================
# CONEXÃO
# ==========================================

print("======================================")
print("       CONTROLE S500 - TECLADO")
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
# ENVIA CONTROLE
# ==========================================

def enviar_controle(x, y, z, r):

    veiculo.mav.manual_control_send(
        veiculo.target_system,
        x,
        y,
        z,
        r,
        0
    )


# ==========================================
# CONTROLE DO TECLADO
# ==========================================

def controle_tela(tela):

    curses.curs_set(0)

    tela.nodelay(True)

    tela.keypad(True)

    x = 0
    y = 0
    z = NEUTRO
    r = 0

    while True:

        tecla = tela.getch()

        # ------------------------------
        # NENHUMA TECLA
        # ------------------------------

        if tecla == -1:

            enviar_controle(
                x,
                y,
                z,
                r
            )

            time.sleep(0.05)

            continue


        # ------------------------------
        # W - FRENTE
        # ------------------------------

        elif tecla in [ord('w'), ord('W')]:

            x = VELOCIDADE


        # ------------------------------
        # S - TRÁS
        # ------------------------------

        elif tecla in [ord('s'), ord('S')]:

            x = -VELOCIDADE


        # ------------------------------
        # A - ESQUERDA
        # ------------------------------

        elif tecla in [ord('a'), ord('A')]:

            y = -VELOCIDADE


        # ------------------------------
        # D - DIREITA
        # ------------------------------

        elif tecla in [ord('d'), ord('D')]:

            y = VELOCIDADE


        # ------------------------------
        # SETA PARA CIMA
        # ------------------------------

        elif tecla == curses.KEY_UP:

            z = 700


        # ------------------------------
        # SETA PARA BAIXO
        # ------------------------------

        elif tecla == curses.KEY_DOWN:

            z = 300


        # ------------------------------
        # SETA ESQUERDA
        # ------------------------------

        elif tecla == curses.KEY_LEFT:

            r = -VELOCIDADE


        # ------------------------------
        # SETA DIREITA
        # ------------------------------

        elif tecla == curses.KEY_RIGHT:

            r = VELOCIDADE


        # ------------------------------
        # Q - PARAR
        # ------------------------------

        elif tecla in [ord('q'), ord('Q')]:

            x = 0
            y = 0
            z = NEUTRO
            r = 0


        # ------------------------------
        # X - SAIR
        # ------------------------------

        elif tecla in [ord('x'), ord('X')]:

            break


        # ------------------------------
        # ENVIA COMANDO
        # ------------------------------

        enviar_controle(
            x,
            y,
            z,
            r
        )


        # ------------------------------
        # MOSTRA STATUS
        # ------------------------------

        tela.clear()

        tela.addstr(
            0,
            0,
            "========================================"
        )

        tela.addstr(
            1,
            0,
            "       CONTROLE DO S500"
        )

        tela.addstr(
            2,
            0,
            "========================================"
        )

        tela.addstr(
            4,
            0,
            f"X/Pitch : {x}"
        )

        tela.addstr(
            5,
            0,
            f"Y/Roll  : {y}"
        )

        tela.addstr(
            6,
            0,
            f"Z/Thrust: {z}"
        )

        tela.addstr(
            7,
            0,
            f"R/Yaw   : {r}"
        )

        tela.addstr(
            9,
            0,
            "W/S = Frente/Tras"
        )

        tela.addstr(
            10,
            0,
            "A/D = Esquerda/Direita"
        )

        tela.addstr(
            11,
            0,
            "↑/↓ = Subir/Descer"
        )

        tela.addstr(
            12,
            0,
            "←/→ = Girar"
        )

        tela.addstr(
            14,
            0,
            "Q = PARAR"
        )

        tela.addstr(
            15,
            0,
            "X = SAIR"
        )

        tela.refresh()


# ==========================================
# INICIAR CONTROLE
# ==========================================

print("\nIniciando controle...")

time.sleep(1)

curses.wrapper(controle_tela)


# ==========================================
# FINAL
# ==========================================

enviar_controle(
    0,
    0,
    500,
    0
)

print("\nControle encerrado.")
