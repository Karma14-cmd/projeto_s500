
from pymavlink import mavutil
import time

print("===================================")
print("     MISSÃO AUTÔNOMA - S500")
print("===================================")

# Conecta ao ArduPilot SITL
veiculo = mavutil.mavlink_connection(
    "udp:127.0.0.1:14550"
)
print("\nAguardando heartbeat...")
veiculo.wait_heartbeat()

print("Heartbeat recebido!")
print("Sistema:", veiculo.target_system)
print("Componente:", veiculo.target_component)

# -----------------------------------
# Mudar para modo GUIDED
# -----------------------------------

print("\nMudando para GUIDED...")

veiculo.set_mode_apm("GUIDED")

time.sleep(2)

# -----------------------------------
# ARMAR
# -----------------------------------

print("Armando o S500...")

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

# -----------------------------------
# DECOLAGEM
# -----------------------------------

altura = 5

print(f"\nDecolando para {altura} metros...")

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
    altura
)

# -----------------------------------
# MONITORAR ALTITUDE
# -----------------------------------

print("\nSubindo...")

while True:

    mensagem = veiculo.recv_match(
        type="GLOBAL_POSITION_INT",
        blocking=True
    )

    altitude = mensagem.relative_alt / 1000.0

    print(f"Altitude: {altitude:.2f} m")

    if altitude >= 4.5:
        break

print("\n===================================")
print("      ALTITUDE DESEJADA ATINGIDA")
print("===================================")

print("\nMantendo voo por 10 segundos...")

time.sleep(10)

# -----------------------------------
# POUSO
# -----------------------------------

print("\nIniciando pouso...")

veiculo.set_mode_apm("LAND")

# -----------------------------------
# ESPERAR POUSO
# -----------------------------------

print("Aguardando pouso...")

while True:

    mensagem = veiculo.recv_match(
        type="GLOBAL_POSITION_INT",
        blocking=True
    )

    altitude = mensagem.relative_alt / 1000.0

    print(f"Altitude: {altitude:.2f} m")

    if altitude <= 0.3:
        break

print("\nS500 pousou!")

# -----------------------------------
# DESARMAR
# -----------------------------------

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
    0
)

print("\n===================================")
print("       MISSÃO FINALIZADA")
print("===================================")
