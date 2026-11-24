"""Auditoria MAVLink e missao curta para ArduCopter SITL; nao usar em aeronave real."""
import argparse
import json
import math
from pathlib import Path
import time


class Session:
    def __init__(self, connection, mavlink, log):
        self.c, self.m, self.log = connection, mavlink, log
        self.system = self.component = None
        self.last_tx = 0
        self.last_hb = time.monotonic()
        self.cache = {}
        self.guard = False
        self.heartbeat_enabled = True
        self.previous_mode = None

    def record(self, event, **fields):
        row = dict(utc=time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
                   event=event, **fields)
        print(json.dumps(row, ensure_ascii=False), flush=True)
        self.log.write(json.dumps(row, ensure_ascii=False) + '\n')
        self.log.flush()

    def receive(self):
        now = time.monotonic()
        if self.heartbeat_enabled and now - self.last_tx >= 1:
            self.c.mav.heartbeat_send(self.m.MAV_TYPE_GCS,
                                     self.m.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            self.last_tx = now
        msg = self.c.recv_match(blocking=True, timeout=0.2)
        if msg is not None:
            if self.system is None:
                if (msg.get_type() == 'HEARTBEAT'
                        and msg.autopilot == self.m.MAV_AUTOPILOT_ARDUPILOTMEGA
                        and msg.type == self.m.MAV_TYPE_QUADROTOR):
                    self.system, self.component = msg.get_srcSystem(), msg.get_srcComponent()
                    self.c.target_system, self.c.target_component = self.system, self.component
                else:
                    return None
            if (msg.get_srcSystem(), msg.get_srcComponent()) != (self.system, self.component):
                return None
            self.cache[msg.get_type()] = (time.monotonic(), msg)
            if msg.get_type() == 'HEARTBEAT':
                self.last_hb = time.monotonic()
                if msg.custom_mode != self.previous_mode:
                    self.record('modo', mode=msg.custom_mode, armed=bool(
                        msg.base_mode & self.m.MAV_MODE_FLAG_SAFETY_ARMED))
                    self.previous_mode = msg.custom_mode
                if self.guard and msg.custom_mode != 4:  # GUIDED
                    raise RuntimeError('GUIDED interrompido; nao retomar a missao automaticamente')
            if msg.get_type() in ('STATUSTEXT', 'COMMAND_ACK', 'FENCE_STATUS'):
                self.record('mavlink', message=msg.to_dict())
        if self.system is not None and time.monotonic() - self.last_hb > 5:
            raise TimeoutError('Heartbeat da aeronave ausente por mais de 5 s')
        return msg

    def wait(self, predicate, timeout, description):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            msg = self.receive()
            if msg is not None and predicate(msg):
                return msg
        raise TimeoutError(description)

    def command(self, command, *params):
        self.c.mav.command_long_send(self.system, self.component, command, 0,
                                     *(list(params) + [0] * (7 - len(params))))
        ack = self.wait(lambda x: x.get_type() == 'COMMAND_ACK' and x.command == command
                        and x.result != self.m.MAV_RESULT_IN_PROGRESS, 15,
                        'Sem ACK final para comando ' + str(command))
        if ack.result != self.m.MAV_RESULT_ACCEPTED:
            raise RuntimeError('Comando recusado: ' + str(ack.to_dict()))

    def param(self, name, optional=False):
        for _ in range(2):
            self.c.mav.param_request_read_send(self.system, self.component, name.encode(), -1)
            try:
                msg = self.wait(lambda x: x.get_type() == 'PARAM_VALUE'
                                and normalize_id(x.param_id) == name, 3, 'Parametro ' + name)
                return msg.param_value
            except TimeoutError:
                pass
        if optional:
            return None
        raise RuntimeError('Parametro indisponivel: ' + name)

    def mode(self, number):
        self.command(self.m.MAV_CMD_DO_SET_MODE, self.m.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, number)
        self.wait(lambda x: x.get_type() == 'HEARTBEAT' and x.custom_mode == number,
                  10, 'Modo nao confirmado')

    def set_param(self, name, value):
        self.c.mav.param_set_send(self.system, self.component, name.encode(),
                                 value, self.m.MAV_PARAM_TYPE_REAL32)
        self.wait(lambda x: x.get_type() == 'PARAM_VALUE'
                  and normalize_id(x.param_id) == name
                  and abs(x.param_value - value) < 0.001, 5, 'Escrita nao confirmada: ' + name)
        actual = self.param(name)
        if abs(actual - value) > 0.001:
            raise RuntimeError('Leitura divergente: ' + name)
        self.record('parametro_confirmado', nome=name, valor=actual)


def normalize_id(value):
    if isinstance(value, bytes):
        value = value.decode('ascii')
    return value.rstrip('\0')


def validate_plan(params, altitude, distance):
    if not math.isfinite(altitude) or not 1 <= altitude <= 5:
        raise ValueError('Altitude deve estar entre 1 e 5 m')
    if not math.isfinite(distance) or not 0 < distance <= 5:
        raise ValueError('Distancia deve ser maior que zero e no maximo 5 m')
    if params['FENCE_ENABLE'] != 1 or not int(params['FENCE_TYPE']) & 1:
        raise ValueError('Geofence de altitude nao esta ativo')
    if params['FENCE_ACTION'] == 0:
        raise ValueError('Geofence configurado apenas para informar')
    if altitude > params['FENCE_ALT_MAX'] - max(1, params['FENCE_MARGIN']):
        raise ValueError('Altitude sem margem suficiente abaixo do teto')
    if params['SIM_RC_FAIL'] != 0:
        raise ValueError('Restaurar SIM_RC_FAIL=0 antes da missao')
    if params['FS_GCS_ENABLE'] != 1 or params['FS_THR_ENABLE'] == 0:
        raise ValueError('Failsafes GCS/RC diferentes do esperado')
    checks = (params.get('ARMING_CHECK') == 1 if params.get('ARMING_CHECK') is not None
              else params.get('ARMING_SKIPCHK') == 0)
    if not checks:
        raise ValueError('Verificacoes de armamento devem permanecer todas habilitadas')


def fly(s, p, altitude, distance):
    validate_plan(p, altitude, distance)
    hb = s.wait(lambda x: x.get_type() == 'HEARTBEAT', 5, 'Sem heartbeat')
    if hb.base_mode & s.m.MAV_MODE_FLAG_SAFETY_ARMED:
        raise RuntimeError('Aeronave ja armada; iniciar somente desarmada')
    # A existencia de SIM_SPEEDUP e SIM_RC_FAIL distingue a configuracao SITL esperada.
    s.param('SIM_SPEEDUP')
    s.command(s.m.MAV_CMD_SET_MESSAGE_INTERVAL, 32, 200000)
    s.command(s.m.MAV_CMD_SET_MESSAGE_INTERVAL, 33, 200000)
    s.wait(lambda x: x.get_type() == 'GLOBAL_POSITION_INT', 10, 'Sem posicao global')
    s.wait(lambda x: x.get_type() == 'LOCAL_POSITION_NED', 10, 'Sem posicao local')
    s.mode(4)
    s.guard = True
    armed_by_us = False
    try:
        # Inclui recusas ou perda de ACK do armamento no tratamento de emergencia.
        armed_by_us = True
        s.command(s.m.MAV_CMD_COMPONENT_ARM_DISARM, 1)
        s.wait(lambda x: x.get_type() == 'HEARTBEAT'
               and x.base_mode & s.m.MAV_MODE_FLAG_SAFETY_ARMED, 10, 'Armamento nao confirmado')
        s.record('armado')
        s.command(s.m.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, altitude)
        s.wait(lambda x: x.get_type() == 'GLOBAL_POSITION_INT'
               and abs(x.relative_alt / 1000 - altitude) <= 0.4, 45, 'Altitude nao atingida')
        origin = s.wait(lambda x: x.get_type() == 'LOCAL_POSITION_NED', 5, 'Sem posicao atual')
        target = (origin.x + distance, origin.y, origin.z)
        s.c.mav.set_position_target_local_ned_send(
            0, s.system, s.component, s.m.MAV_FRAME_LOCAL_NED, 3576,
            *target, 0, 0, 0, 0, 0, 0, 0, 0)
        s.record('alvo_navegacao', north=target[0], east=target[1], down=target[2])
        # Setpoints de posicao nao possuem COMMAND_ACK; validar pela telemetria.
        s.wait(lambda x: x.get_type() == 'LOCAL_POSITION_NED'
               and math.dist((x.x, x.y, x.z), target) <= 0.6, 40, 'Alvo nao atingido')
        s.record('navegacao_confirmada')
        s.guard = False
        s.mode(9)  # LAND
        s.wait(lambda x: x.get_type() == 'HEARTBEAT'
               and not x.base_mode & s.m.MAV_MODE_FLAG_SAFETY_ARMED, 90, 'Sem desarmamento no pouso')
        s.record('missao_concluida', criterio='Altitude, alvo e desarmamento confirmados')
    except BaseException:
        s.guard = False
        if armed_by_us:
            try:
                hb = s.wait(lambda x: x.get_type() == 'HEARTBEAT', 3, 'Sem heartbeat para emergencia')
                if hb.base_mode & s.m.MAV_MODE_FLAG_SAFETY_ARMED:
                    if hb.custom_mode == 4:
                        s.mode(9)
                        s.record('emergencia_land_enviado')
                    else:
                        s.record('modo_externo_preservado', mode=hb.custom_mode)
                    s.wait(lambda x: x.get_type() == 'HEARTBEAT'
                           and not x.base_mode & s.m.MAV_MODE_FLAG_SAFETY_ARMED,
                           90, 'Pouso de emergencia nao confirmado')
            except Exception as exc:
                s.record('emergencia_nao_confirmada', erro=str(exc))
        raise


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--connection', default='udpin:127.0.0.1:14551')
    parser.add_argument('--fly-sitl', action='store_true', help='Executar missao somente no SITL')
    parser.add_argument('--altitude', type=float, default=5)
    parser.add_argument('--distance', type=float, default=2)
    parser.add_argument('--log', default='validacao_mavlink.jsonl')
    args = parser.parse_args()
    from pymavlink import mavutil
    with Path(args.log).open('a', encoding='utf-8') as log:
        c = mavutil.mavlink_connection(args.connection, source_system=255, source_component=190)
        s = Session(c, mavutil.mavlink, log)
        try:
            s.wait(lambda x: x.get_type() == 'HEARTBEAT', 20, 'SITL nao encontrado')
            names = ('FENCE_ENABLE', 'FENCE_TYPE', 'FENCE_ALT_MAX', 'FENCE_ACTION',
                     'FENCE_MARGIN', 'FS_THR_ENABLE', 'FS_GCS_ENABLE')
            params = {name: s.param(name) for name in names}
            for name in ('SIM_RC_FAIL', 'SIM_SPEEDUP', 'FS_GCS_TIMEOUT', 'FS_OPTIONS',
                         'ARMING_CHECK', 'ARMING_SKIPCHK', 'RTL_ALT', 'RTL_ALT_M',
                         'RTL_CLIMB_MIN', 'RTL_CLIMB_MIN_M', 'SYSID_MYGCS', 'MAV_GCS_SYSID'):
                params[name] = s.param(name, optional=True)
            s.record('auditoria_parametros', parametros=params,
                     system=s.system, component=s.component)
            if args.fly_sitl:
                fly(s, params, args.altitude, args.distance)
            else:
                s.record('auditoria_concluida', voo_executado=False)
        except (Exception, KeyboardInterrupt) as exc:
            s.record('erro', tipo=type(exc).__name__, mensagem=str(exc))
            return 1
        finally:
            c.close()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
