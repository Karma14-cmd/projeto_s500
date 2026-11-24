"""Inicialização com prazos e telemetria para o SITL local."""
import time
from pymavlink import mavutil


def heartbeat(veiculo):
    mav = mavutil.mavlink
    veiculo.mav.heartbeat_send(mav.MAV_TYPE_GCS,mav.MAV_AUTOPILOT_INVALID,0,0,0)


def aguardar(veiculo, descricao, condicao, prazo=45):
    limite=time.monotonic()+prazo
    ultimo=0
    while time.monotonic()<limite:
        if time.monotonic()-ultimo>1:
            heartbeat(veiculo)
            ultimo=time.monotonic()
        msg=veiculo.recv_match(blocking=True,timeout=.2)
        if msg and msg.get_type()=='STATUSTEXT':
            print(msg.text,flush=True)
        if msg and condicao(msg):
            return msg
    raise TimeoutError(f'Tempo esgotado: {descricao}')


def conectar_e_decolar(conexao, altura):
    """Prepara o voo simulado; não força armamento nem desabilita verificações."""
    if conexao not in ('udp:127.0.0.1:14550','udpin:127.0.0.1:14550'):
        raise ValueError('Este inicializador é restrito ao SITL local em UDP 14550.')
    m=mavutil.mavlink_connection(conexao)
    mav=mavutil.mavlink
    print('Aguardando SITL local...',flush=True)
    if m.wait_heartbeat(timeout=30) is None:
        m.close()
        raise TimeoutError('Sem heartbeat: inicie gazebo/windows/iniciar_sitl.py')
    m.mav.request_data_stream_send(m.target_system,m.target_component,mav.MAV_DATA_STREAM_ALL,10,1)
    try:
        print('Aguardando estimador de posição/GPS...',flush=True)
        aguardar(m,'posição válida no EKF',lambda x:x.get_type()=='EKF_STATUS_REPORT' and x.flags&19==19,120)
        m.set_mode_apm('GUIDED')
        aguardar(m,'modo GUIDED',lambda x:x.get_type()=='HEARTBEAT' and m.flightmode=='GUIDED')
        m.mav.command_long_send(m.target_system,m.target_component,mav.MAV_CMD_COMPONENT_ARM_DISARM,0,1,0,0,0,0,0,0)
        aguardar(m,'armamento',lambda x:x.get_type()=='HEARTBEAT' and m.motors_armed(),30)
        print(f'Decolando para {altura:.1f} m...',flush=True)
        m.mav.command_long_send(m.target_system,m.target_component,mav.MAV_CMD_NAV_TAKEOFF,0,0,0,0,0,0,0,altura)
        aguardar(m,'decolagem',lambda x:x.get_type()=='GLOBAL_POSITION_INT' and x.relative_alt/1000>=altura-.2,60)
        return m
    except Exception:
        m.set_mode_apm('LAND')
        m.close()
        raise
