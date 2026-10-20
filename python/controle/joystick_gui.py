import pygame
from pymavlink import mavutil
import time
import math

# ============================================================
# CONFIGURAÇÕES
# ============================================================

CONEXAO = "udp:127.0.0.1:14550"

ALTURA_DECOLAGEM = 2.0

MAX_VX = 2.0       # frente/trás - m/s
MAX_VY = 2.0       # esquerda/direita - m/s
MAX_VZ = 1.0       # subida/descida - m/s
MAX_YAW = 1.2      # giro - rad/s

ACELERACAO = 2.5
FPS = 50

# ============================================================
# ESTADO DO DRONE
# ============================================================

vx = 0.0
vy = 0.0
vz = 0.0
yaw_rate = 0.0

altitude = 0.0

voando = True
pousando = False


# ============================================================
# FUNÇÃO DE ACELERAÇÃO SUAVE
# ============================================================

def aproximar(valor, alvo, velocidade, dt):

    diferenca = alvo - valor

    limite = velocidade * dt

    if abs(diferenca) <= limite:
        return alvo

    if diferenca > 0:
        return valor + limite

    return valor - limite


# ============================================================
# CONEXÃO COM ARDUPILOT
# ============================================================

print("==========================================")
print("      S500 - JOYSTICK GRÁFICO")
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
# AGUARDAR ALTURA
# ============================================================

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

        if altitude >= ALTURA_DECOLAGEM - 0.2:
            break


print("\n\nALTURA ATINGIDA!")

time.sleep(2)


# ============================================================
# INICIALIZAR PYGAME
# ============================================================

pygame.init()

largura = 900
altura_janela = 650

tela = pygame.display.set_mode(
    (largura, altura_janela)
)

pygame.display.set_caption(
    "S500 - Joystick Virtual"
)

relogio = pygame.time.Clock()


# ============================================================
# FONTES
# ============================================================

fonte_titulo = pygame.font.Font(
    None,
    36
)

fonte = pygame.font.Font(
    None,
    26
)

fonte_pequena = pygame.font.Font(
    None,
    22
)


# ============================================================
# ENVIA VELOCIDADE MAVLINK
# ============================================================

def enviar_velocidade():

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
        yaw_rate
    )


# ============================================================
# DESENHAR JOYSTICK
# ============================================================

def desenhar_joystick():

    centro_x = 230
    centro_y = 350

    raio = 130

    # Área do joystick

    pygame.draw.circle(
        tela,
        (60, 60, 60),
        (centro_x, centro_y),
        raio,
        4
    )

    pygame.draw.line(
        tela,
        (80, 80, 80),
        (centro_x - raio, centro_y),
        (centro_x + raio, centro_y),
        2
    )

    pygame.draw.line(
        tela,
        (80, 80, 80),
        (centro_x, centro_y - raio),
        (centro_x, centro_y + raio),
        2
    )

    # Normalizar posição do joystick

    jx = vy / MAX_VY
    jy = -vx / MAX_VX

    jx = max(-1, min(1, jx))
    jy = max(-1, min(1, jy))

    pos_x = centro_x + int(jx * raio)
    pos_y = centro_y + int(jy * raio)

    pygame.draw.circle(
        tela,
        (220, 220, 220),
        (pos_x, pos_y),
        25
    )

    pygame.draw.circle(
        tela,
        (255, 255, 255),
        (pos_x, pos_y),
        25,
        3
    )

    texto = fonte.render(
        "MOVIMENTO",
        True,
        (255, 255, 255)
    )

    tela.blit(
        texto,
        (centro_x - 65, centro_y + raio + 25)
    )


# ============================================================
# DESENHAR YAW
# ============================================================

def desenhar_yaw():

    centro_x = 650
    centro_y = 350

    raio = 100

    pygame.draw.circle(
        tela,
        (60, 60, 60),
        (centro_x, centro_y),
        raio,
        4
    )

    # posição do yaw

    deslocamento = yaw_rate / MAX_YAW

    deslocamento = max(
        -1,
        min(1, deslocamento)
    )

    pos_x = centro_x + int(
        deslocamento * raio
    )

    pygame.draw.circle(
        tela,
        (220, 220, 220),
        (pos_x, centro_y),
        22
    )

    texto = fonte.render(
        "YAW",
        True,
        (255, 255, 255)
    )

    tela.blit(
        texto,
        (centro_x - 30, centro_y + raio + 25)
    )


# ============================================================
# LOOP PRINCIPAL
# ============================================================

executando = True

ultimo_tempo = time.monotonic()

while executando and voando:

    agora = time.monotonic()

    dt = agora - ultimo_tempo

    ultimo_tempo = agora

    if dt > 0.1:
        dt = 0.1


    # ========================================================
    # EVENTOS
    # ========================================================

    for evento in pygame.event.get():

        if evento.type == pygame.QUIT:

            executando = False

            pousando = True


        if evento.type == pygame.KEYDOWN:

            if evento.key == pygame.K_l:

                print("\nPousando...")

                pousando = True

                executando = False


    # ========================================================
    # TECLAS
    # ========================================================

    teclas = pygame.key.get_pressed()


    # ========================================================
    # ALVOS
    # ========================================================

    alvo_vx = 0.0
    alvo_vy = 0.0
    alvo_vz = 0.0
    alvo_yaw = 0.0


    # ========================================================
    # FRENTE / TRÁS
    # ========================================================

    if teclas[pygame.K_w]:

        alvo_vx += MAX_VX

    if teclas[pygame.K_s]:

        alvo_vx -= MAX_VX


    # ========================================================
    # ESQUERDA / DIREITA
    # ========================================================

    if teclas[pygame.K_a]:

        alvo_vy -= MAX_VY

    if teclas[pygame.K_d]:

        alvo_vy += MAX_VY


    # ========================================================
    # SUBIR / DESCER
    # ========================================================

    if teclas[pygame.K_UP]:

        alvo_vz -= MAX_VZ

    if teclas[pygame.K_DOWN]:

        alvo_vz += MAX_VZ


    # ========================================================
    # YAW
    # ========================================================

    if teclas[pygame.K_LEFT]:

        alvo_yaw -= MAX_YAW

    if teclas[pygame.K_RIGHT]:

        alvo_yaw += MAX_YAW


    # ========================================================
    # NORMALIZAR MOVIMENTO DIAGONAL
    # ========================================================

    magnitude = math.sqrt(
        alvo_vx ** 2 +
        alvo_vy ** 2
    )

    if magnitude > MAX_VX:

        fator = MAX_VX / magnitude

        alvo_vx *= fator
        alvo_vy *= fator


    # ========================================================
    # ACELERAÇÃO
    # ========================================================

    vx = aproximar(
        vx,
        alvo_vx,
        ACELERACAO,
        dt
    )

    vy = aproximar(
        vy,
        alvo_vy,
        ACELERACAO,
        dt
    )

    vz = aproximar(
        vz,
        alvo_vz,
        ACELERACAO,
        dt
    )

    yaw_rate = aproximar(
        yaw_rate,
        alvo_yaw,
        ACELERACAO,
        dt
    )


    # ========================================================
    # Q = PARADA
    # ========================================================

    if teclas[pygame.K_q]:

        vx = 0
        vy = 0
        vz = 0
        yaw_rate = 0


    # ========================================================
    # RECEBER TELEMETRIA
    # ========================================================

    msg = veiculo.recv_match(
        type="GLOBAL_POSITION_INT",
        blocking=False
    )

    if msg:

        altitude = (
            msg.relative_alt / 1000.0
        )


    # ========================================================
    # ENVIAR MAVLINK
    # ========================================================

    enviar_velocidade()


    # ========================================================
    # INTERFACE
    # ========================================================

    tela.fill((20, 20, 20))


    titulo = fonte_titulo.render(
        "S500 - JOYSTICK VIRTUAL",
        True,
        (255, 255, 255)
    )

    tela.blit(
        titulo,
        (260, 25)
    )


    status = fonte.render(
        "GUIDED  |  ARMADO",
        True,
        (255, 255, 255)
    )

    tela.blit(
        status,
        (330, 75)
    )


    # Altitude

    texto_altitude = fonte.render(
        f"Altitude: {altitude:.2f} m",
        True,
        (255, 255, 255)
    )

    tela.blit(
        texto_altitude,
        (350, 120)
    )


    # Velocidades

    dados = [
        f"VX: {vx:+.2f} m/s",
        f"VY: {vy:+.2f} m/s",
        f"VZ: {vz:+.2f} m/s",
        f"YAW: {yaw_rate:+.2f} rad/s"
    ]

    for i, texto in enumerate(dados):

        superficie = fonte_pequena.render(
            texto,
            True,
            (255, 255, 255)
        )

        tela.blit(
            superficie,
            (550, 480 + i * 28)
        )


    # Joysticks

    desenhar_joystick()

    desenhar_yaw()


    # Instruções

    instrucoes = [
        "W/S = Frente / Trás",
        "A/D = Esquerda / Direita",
        "↑/↓ = Subir / Descer",
        "←/→ = Girar",
        "Q = Parada",
        "L = Pousar"
    ]

    for i, texto in enumerate(instrucoes):

        superficie = fonte_pequena.render(
            texto,
            True,
            (220, 220, 220)
        )

        tela.blit(
            superficie,
            (40, 80 + i * 30)
        )


    pygame.display.flip()

    relogio.tick(FPS)


# ============================================================
# POUSO
# ============================================================

print("\nComandos de movimento zerados.")

vx = 0
vy = 0
vz = 0
yaw_rate = 0

enviar_velocidade()

time.sleep(0.5)


if pousando:

    print("Mudando para LAND...")

    veiculo.set_mode_apm("LAND")


    # -----------------------------------------------
    # ESPERAR POUSO
    # -----------------------------------------------

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


# ============================================================
# DESARMAR
# ============================================================

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


pygame.quit()


print("\n==========================================")
print("          MISSÃO FINALIZADA")
print("==========================================")
