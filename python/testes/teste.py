from pymavlink import mavutil

print("Conectando ao S500 virtual...")

veiculo = mavutil.mavlink_connection(
    "udp:127.0.0.1:14550"
)

print("Aguardando heartbeat...")

veiculo.wait_heartbeat()

print("================================")
print("HEARTBEAT RECEBIDO!")
print("SISTEMA:", veiculo.target_system)
print("COMPONENTE:", veiculo.target_component)
print("================================")
