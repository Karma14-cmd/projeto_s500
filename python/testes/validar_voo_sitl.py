"""Ensaio integrado de decolagem/movimento/pouso; SOMENTE SITL em loopback."""
import json
import math
from pathlib import Path
import time
from pymavlink import mavutil


def main():
    m = mavutil.mavlink_connection('udpin:127.0.0.1:14550')
    if m.wait_heartbeat(timeout=20) is None:
        raise RuntimeError('SITL sem heartbeat em UDP 14550')
    mav = mavutil.mavlink
    m.mav.request_data_stream_send(m.target_system,m.target_component,mav.MAV_DATA_STREAM_ALL,10,1)
    m.mav.command_long_send(m.target_system,m.target_component,mav.MAV_CMD_SET_MESSAGE_INTERVAL,0,
                            mav.MAVLINK_MSG_ID_DISTANCE_SENSOR,100000,0,0,0,0,0)
    estado={'altitude':None,'roll':0.,'pitch':0.,'armado':False,'rng':None}
    amostras=[]
    ultimo_hb=0.

    def aguardar(nome, duracao, condicao, comando=None):
        nonlocal ultimo_hb
        limite=time.monotonic()+duracao
        while time.monotonic()<limite:
            if time.monotonic()-ultimo_hb>1:
                m.mav.heartbeat_send(mav.MAV_TYPE_GCS,mav.MAV_AUTOPILOT_INVALID,0,0,0)
                ultimo_hb=time.monotonic()
            if comando:
                comando()
            msg=m.recv_match(blocking=True,timeout=.1)
            if msg:
                if msg.get_type()=='GLOBAL_POSITION_INT':
                    estado['altitude']=msg.relative_alt/1000
                    estado['vx']=msg.vx/100;estado['vy']=msg.vy/100
                    amostras.append(dict(etapa=nome,**estado))
                elif msg.get_type()=='ATTITUDE':
                    estado['roll']=msg.roll;estado['pitch']=msg.pitch
                elif msg.get_type()=='DISTANCE_SENSOR':
                    # TF-Luna do Gazebo, recebido pelo ArduPilot via ponte (rng_1).
                    estado['rng']=msg.current_distance/100
                elif msg.get_type()=='RANGEFINDER':
                    estado['rng']=msg.distance
                elif msg.get_type()=='HEARTBEAT':
                    estado['armado']=bool(msg.base_mode&mav.MAV_MODE_FLAG_SAFETY_ARMED)
                elif msg.get_type()=='STATUSTEXT':
                    print(msg.text,flush=True)
                if abs(estado['roll'])>1 or abs(estado['pitch'])>1:
                    raise RuntimeError(f'Inclinação excessiva: {estado}')
            if condicao(estado):
                print(nome,estado,flush=True)
                return
        raise RuntimeError(f'Tempo esgotado em {nome}: {estado}')

    def velocidade(vx=0.):
        m.mav.set_position_target_local_ned_send(0,m.target_system,m.target_component,
            mav.MAV_FRAME_BODY_NED,1479,0,0,0,vx,0,0,0,0,0,0,0)

    sucesso=False
    try:
        m.set_mode_apm('GUIDED')
        aguardar('guided',10,lambda s:m.flightmode=='GUIDED')
        m.mav.command_long_send(m.target_system,m.target_component,mav.MAV_CMD_COMPONENT_ARM_DISARM,0,1,0,0,0,0,0,0)
        aguardar('armado',20,lambda s:s['armado'])
        m.mav.command_long_send(m.target_system,m.target_component,mav.MAV_CMD_NAV_TAKEOFF,0,0,0,0,0,0,0,2.)
        aguardar('decolagem_2m',50,lambda s:s['altitude'] is not None and s['altitude']>1.7)
        inicio=time.monotonic()
        aguardar('avancar',8,lambda s:time.monotonic()-inicio>3,lambda:velocidade(.4))
        inicio=time.monotonic()
        aguardar('parar',10,lambda s:time.monotonic()-inicio>3 and math.hypot(s.get('vx',1),s.get('vy',1))<.25,velocidade)
        # Em voo pairado o TF-Luna fica ~0,10 m abaixo da origem do corpo.
        aguardar('rangefinder',10,lambda s:s['rng'] is not None and s['altitude'] is not None
                 and abs(s['rng']-(s['altitude']+.10))<.30,velocidade)
        m.set_mode_apm('LAND')
        aguardar('pouso_desarmado',70,lambda s:not s['armado'] and s['altitude'] is not None and s['altitude']<.3)
        sucesso=True
    finally:
        if not sucesso:
            m.set_mode_apm('LAND')
        pasta=Path(__file__).resolve().parents[2]/'resultados/logs/sitl'
        pasta.mkdir(parents=True,exist_ok=True)
        (pasta/'voo.json').write_text(json.dumps({'aprovado':sucesso,'amostras':amostras},indent=2),encoding='utf-8')
        m.close()
    print('VOO SITL APROVADO',flush=True)


if __name__=='__main__':
    main()
