from pymavlink import mavutil
import time

CONEXAO = "udp:127.0.0.1:14550"
ALTURA = 2.0

print("===================================")
print("       TESTE DE VOO S500")
print("===================================")

print("\nConectando...")

veiculo = mavutil.mavlink_connection(CONEXAO)

print("Aguardando heartbeat...")

veiculo.wait_heartbeat()

print("HEARTBEAT RECEBIDO!")
print("Sistema:", veiculo.target_system)
print("Componente:", veiculo.target_component)


# GUIDED
print("\nMudando para GUIDED...")

veiculo.set_mode_apm("GUIDED")

time.sleep(2)

print("GUIDED configurado.")


# ARM
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


# TAKEOFF
print("\nDecolando para 2 metros...")

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
    ALTURA
)


# MONITORAR ALTITUDE

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

    if altitude >= 1.7:
        break


print("\n\nALTURA ATINGIDA!")

print("\nS500 permanecerá parado por 5 segundos.")

time.sleep(5)


# LAND

print("\nIniciando pouso...")

veiculo.set_mode_apm("LAND")

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

    if altitude <= 0.3:
        break


print("\n\nS500 pousou!")

time.sleep(2)

print("\nDesarmando...")

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

print("\n===================================")
print("       TESTE FINALIZADO")
print("===================================")
