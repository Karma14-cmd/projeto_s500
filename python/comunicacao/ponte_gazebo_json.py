"""Ponte experimental Gazebo Harmonic <-> ArduPilot JSON, somente loopback.

Usa a física do Gazebo e os motores do S500. Não substitui o autopiloto.
Envia ao SITL a IMU e a pose do Gazebo e, como rangefinder 1 (``rng_1``), a
distância do TF-Luna. GPS, bússola e barômetro do ArduPilot continuam sendo
sintetizados pelo próprio SITL a partir da posição (limite da interface JSON).
Referência: ArduPilot/libraries/SITL/SIM_JSON.cpp e ardupilot_gazebo.
"""
import json
import math
from pathlib import Path
import socket
import struct
import sys
import threading
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

# Valor enviado quando o TF-Luna não tem leitura válida: acima de RNGFND1_MAX,
# o ArduPilot o trata como "fora da faixa" em vez de manter a última medida.
RNG_SEM_LEITURA = 99.0


def quat_para_euler(w, x, y, z):
    """Converte quaternion (w, x, y, z) para roll, pitch, yaw (rad) em convenção ZYX."""
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    seno = max(-1.0, min(1.0, 2 * (w * y - z * x)))
    pitch = math.asin(seno)
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return [roll, pitch, yaw]


def rotacionar(q, v):
    w, x, y, z = q
    vx, vy, vz = v
    tx, ty, tz = 2*(y*vz-z*vy), 2*(z*vx-x*vz), 2*(x*vy-y*vx)
    return [vx+w*tx+y*tz-z*ty, vy+w*ty+z*tx-x*tz, vz+w*tz+x*ty-y*tx]


def decodificar_servo(pacote):
    if len(pacote) not in (40,72):
        return None
    magic,rate,frame=struct.unpack_from('<HHI',pacote)
    if (magic,len(pacote)) not in ((18458,40),(29569,72)):
        return None
    return struct.unpack_from('<'+'H'*((len(pacote)-8)//2),pacote,8)


def velocidades_motores(pwm):
    return [max(0.,min(1.,(p-1000)/1000))*1100 for p in pwm[:4]]


def main():
    from gz.transport13 import Node
    from gz.msgs10.imu_pb2 import IMU
    from gz.msgs10.odometry_pb2 import Odometry
    from gz.msgs10.actuators_pb2 import Actuators
    from gz.msgs10.laserscan_pb2 import LaserScan
    from controle.leitura_infravermelho_s500 import distancia_valida
    node = Node()
    pub = node.advertise('/S500/gazebo/command/motor_speed', Actuators)
    lock = threading.Lock()
    estado = {'imu':None, 'odom':None, 'recebido':0., 'rng':None}

    def imu(m):
        with lock:
            estado['imu'] = m
            estado['recebido'] = time.monotonic()

    def odom(m):
        with lock:
            estado['odom'] = m

    def rng(m):
        distancia = distancia_valida(m)
        with lock:
            estado['rng'] = distancia

    node.subscribe(IMU, '/s500/imu', imu)
    node.subscribe(Odometry, '/model/quadricoptero/odometry', odom)
    node.subscribe(LaserScan, '/s500/infravermelho/scan', rng)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('127.0.0.1', 9002))
    sock.settimeout(.5)
    print('Ponte JSON: 127.0.0.1:9002; aguardando SITL e Gazebo.', flush=True)
    ultimo_t = -1.
    ultimo_log = 0.
    try:
        while True:
            try:
                pacote, endereco = sock.recvfrom(4096)
            except socket.timeout:
                pub.publish(Actuators(velocity=[0]*4))
                continue
            pwm=decodificar_servo(pacote)
            if pwm is None:
                continue
            velocidades = velocidades_motores(pwm)
            pub.publish(Actuators(velocity=velocidades))
            # Aguarda um estado novo. Não avança o tempo do SITL com dados velhos.
            limite = time.monotonic()+2
            while time.monotonic()<limite:
                with lock:
                    im, od, recebido, distancia = estado['imu'],estado['odom'],estado['recebido'],estado['rng']
                if im is not None and od is not None:
                    t = im.header.stamp.sec+im.header.stamp.nsec/1e9
                    if t>ultimo_t and time.monotonic()-recebido<1:
                        break
                time.sleep(.001)
            else:
                pub.publish(Actuators(velocity=[0]*4))
                continue
            ultimo_t=t
            p,q,v=od.pose.position,od.pose.orientation,od.twist.linear
            velocidade = rotacionar([q.w,q.x,q.y,q.z],[v.x,v.y,v.z])
            g,a=im.angular_velocity,im.linear_acceleration
            dados={'timestamp':t,'imu':{'gyro':[g.x,-g.y,-g.z],
                   'accel_body':[a.x,-a.y,-a.z]},'position':[p.x,-p.y,-p.z],
                   'quaternion':[q.w,q.x,-q.y,-q.z],
                   'velocity':[velocidade[0],-velocidade[1],-velocidade[2]],
                   'no_time_sync':True}
            # Contorno para um bug de índice no SIM_JSON.cpp do CopterStable
            # (commit fbd0a316): o bit de rng_1 é 1<<10, mas o laço em
            # SIM_JSON.cpp testa bits 7..12 com rangefinder_m[i-7] = state.rng[i-7].
            # O bit 7 é o campo "attitude" (Euler). Enviando "attitude" junto com
            # "quaternion" (que continua prevalecendo para a rotação) e rng_1..rng_4,
            # rangefinder_m[0] (RNGFND1) é preenchido tanto neste binário quanto
            # em versões corrigidas do ArduPilot.
            dados['attitude'] = quat_para_euler(*dados['quaternion'])
            leitura_rng = RNG_SEM_LEITURA if distancia is None else distancia
            for chave in ('rng_1', 'rng_2', 'rng_3', 'rng_4'):
                dados[chave] = leitura_rng
            if all(math.isfinite(v) for v in dados['position']+dados['velocity']+dados['quaternion']+dados['attitude']+dados['imu']['accel_body']+dados['imu']['gyro']):
                sock.sendto(('\n'+json.dumps(dados,separators=(',',':'),allow_nan=False)+'\n').encode(),endereco)
            if time.monotonic()-ultimo_log>2:
                print(f't={t:.2f} z={p.z:.2f} rng={dados["rng_1"]:.2f} PWM={pwm[:4]}',flush=True)
                ultimo_log=time.monotonic()
    finally:
        pub.publish(Actuators(velocity=[0]*4))
        sock.close()


if __name__ == '__main__':
    main()
