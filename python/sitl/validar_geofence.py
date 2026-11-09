"""Teste de teto de 7 m por tentativa de decolagem a 10 m, somente no SITL."""
import argparse
import time
from pathlib import Path
from pymavlink import mavutil
from voo_autonomo import Session


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--connection', default='tcp:127.0.0.1:5762')
    parser.add_argument('--log', default='geofence.jsonl')
    args = parser.parse_args()
    with Path(args.log).open('a', encoding='utf-8') as log:
        c = mavutil.mavlink_connection(args.connection, source_system=255, source_component=190)
        s = Session(c, mavutil.mavlink, log)
        try:
            hb = s.wait(lambda x: x.get_type() == 'HEARTBEAT', 15, 'Sem SITL')
            if hb.base_mode & 128:
                raise RuntimeError('Iniciar desarmado')
            s.param('SIM_SPEEDUP')
            if s.param('FENCE_ENABLE') != 1 or s.param('FENCE_ALT_MAX') != 7:
                raise RuntimeError('Teste exige fence ativo com teto 7 m')
            s.command(mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 33, 200000)
            s.command(mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 162, 200000)
            s.mode(4)
            s.command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)
            s.wait(lambda x: x.get_type() == 'HEARTBEAT' and x.base_mode & 128,
                   10, 'Sem armamento')
            s.command(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 10)
            s.record('decolagem_10m_solicitada', teto_m=7)
            end = time.monotonic() + 60
            breached = reacted = False
            peak = 0
            while time.monotonic() < end:
                msg = s.receive()
                if msg is None:
                    continue
                if msg.get_type() == 'GLOBAL_POSITION_INT':
                    peak = max(peak, msg.relative_alt / 1000)
                if msg.get_type() == 'FENCE_STATUS' and msg.breach_status:
                    breached = True
                if msg.get_type() == 'HEARTBEAT':
                    reacted |= msg.custom_mode in (6, 9)
                    if not msg.base_mode & 128:
                        if breached and reacted:
                            s.record('geofence_aprovado', pico_altitude_m=peak,
                                     violacao_confirmada=True, reacao_rtl_ou_land=True,
                                     desarmado=True)
                            return 0
                        raise RuntimeError('Pouso ocorreu sem evidencia suficiente de violacao/reacao')
            raise TimeoutError('Teste de geofence sem conclusao')
        except (Exception, KeyboardInterrupt) as exc:
            s.record('geofence_erro', erro=str(exc))
            try:
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
            c.close()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
