"""Configurar e testar os failsafes RC/GCS exclusivamente em ArduCopter SITL."""
import argparse
from pathlib import Path
import time
from pymavlink import mavutil
from voo_autonomo import Session


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--connection', default='tcp:127.0.0.1:5762')
    parser.add_argument('--test', choices=['configure', 'rc', 'gcs'], required=True)
    parser.add_argument('--log', default='fase3.jsonl')
    args = parser.parse_args()
    source = 254 if args.test == 'gcs' else 255
    with Path(args.log).open('a', encoding='utf-8') as log:
        c = mavutil.mavlink_connection(args.connection, source_system=source, source_component=190)
        s = Session(c, mavutil.mavlink, log)
        gcs_key = None
        original_gcs = None
        try:
            hb = s.wait(lambda x: x.get_type() == 'HEARTBEAT', 15, 'Sem SITL')
            if hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                raise RuntimeError('Iniciar desarmado')
            s.param('SIM_SPEEDUP')
            before = {}
            config = dict(FENCE_ENABLE=1, FENCE_ALT_MAX=7, FENCE_ACTION=1,
                          FENCE_MARGIN=2, FS_THR_ENABLE=1, FS_GCS_ENABLE=1,
                          FS_GCS_TIMEOUT=5, FS_OPTIONS=0, SIM_RC_FAIL=0)
            # Mantem outros tipos de fence e acrescenta o teto.
            config['FENCE_TYPE'] = int(s.param('FENCE_TYPE')) | 1
            for modern, legacy, modern_value, legacy_value in [
                ('RTL_ALT_M', 'RTL_ALT', 5, 500),
                ('RTL_CLIMB_MIN_M', 'RTL_CLIMB_MIN', 0, 0),
                ('ARMING_SKIPCHK', 'ARMING_CHECK', 0, 1),
                ('WP_SPD_UP', 'WPNAV_SPEED_UP', 0.5, 50),
                ('WP_SPD_DN', 'WPNAV_SPEED_DN', 0.5, 50),
                ('WP_SPD', 'WPNAV_SPEED', 1, 100),
            ]:
                key = modern if s.param(modern, optional=True) is not None else legacy
                config[key] = modern_value if key == modern else legacy_value
            for key, value in config.items():
                before[key] = s.param(key)
            s.record('configuracao_anterior', parametros=before)
            for key, value in config.items():
                s.set_param(key, value)
            if args.test == 'configure':
                s.record('configuracao_concluida', testes_em_voo=False)
                return 0
            if args.test == 'gcs':
                gcs_key = ('MAV_GCS_SYSID' if s.param('MAV_GCS_SYSID', optional=True)
                           is not None else 'SYSID_MYGCS')
                original_gcs = s.param(gcs_key)
                high = s.param('MAV_GCS_SYSID_HI', optional=True)
                if high is not None and high >= 254:
                    raise RuntimeError('Intervalo de IDs GCS impede isolamento do teste')
                s.set_param(gcs_key, 254)
            s.command(mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 33, 200000)
            s.mode(4)
            s.command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)
            s.wait(lambda x: x.get_type() == 'HEARTBEAT'
                   and x.base_mode & 128, 10, 'Sem armamento')
            s.command(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 3)
            altitude = s.wait(lambda x: x.get_type() == 'GLOBAL_POSITION_INT'
                             and x.relative_alt >= 2700, 45, 'Sem decolagem')
            s.record('decolagem_confirmada', altitude_m=altitude.relative_alt/1000)
            if args.test == 'rc':
                s.set_param('SIM_RC_FAIL', 1)
                s.record('falha_rc_injetada')
            else:
                # Mission Planner continua em 255; so 254 e monitorado durante este teste.
                s.heartbeat_enabled = False
                s.record('heartbeats_gcs_interrompidos', source_system=254)
            start = time.monotonic()
            s.wait(lambda x: x.get_type() == 'HEARTBEAT' and x.custom_mode == 6,
                   20, 'RTL nao confirmado')
            s.record('failsafe_rtl_confirmado', teste=args.test,
                     segundos_apos_injecao=round(time.monotonic()-start, 3))
            if args.test == 'rc':
                s.set_param('SIM_RC_FAIL', 0)
            else:
                s.heartbeat_enabled = True
            s.wait(lambda x: x.get_type() == 'HEARTBEAT' and not x.base_mode & 128,
                   90, 'Pouso/desarmamento nao confirmado')
            s.record('teste_fase3_aprovado', teste=args.test, desarmado=True)
            return 0
        except (Exception, KeyboardInterrupt) as exc:
            s.record('teste_fase3_erro', teste=args.test, erro=str(exc))
            try:
                s.heartbeat_enabled = True
                s.set_param('SIM_RC_FAIL', 0)
                hb = s.wait(lambda x: x.get_type() == 'HEARTBEAT', 5, 'Sem heartbeat')
                if hb.base_mode & 128:
                    if hb.custom_mode == 4:
                        s.mode(9)
                    s.wait(lambda x: x.get_type() == 'HEARTBEAT' and not x.base_mode & 128,
                           90, 'Pouso nao confirmado')
            except Exception as emergency:
                s.record('emergencia_nao_confirmada', erro=str(emergency))
            return 1
        finally:
            if original_gcs is not None:
                s.heartbeat_enabled = True
                s.set_param(gcs_key, original_gcs)
            c.close()


if __name__ == '__main__':
    raise SystemExit(main())
