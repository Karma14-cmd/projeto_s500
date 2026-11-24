"""Exporta parametros e estado atual do SITL, sem alterar configuracoes."""
import argparse
import json
from pathlib import Path
import time
from pymavlink import mavutil
from voo_autonomo import Session


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--connection', default='tcp:127.0.0.1:5762')
    parser.add_argument('--output', default='parametros_finais.json')
    args = parser.parse_args()
    target = Path(args.output)
    with target.with_suffix('.jsonl').open('a', encoding='utf-8') as log:
        c = mavutil.mavlink_connection(args.connection, source_system=255, source_component=190)
        s = Session(c, mavutil.mavlink, log)
        try:
            s.wait(lambda x: x.get_type() == 'HEARTBEAT', 15, 'Sem SITL')
            c.mav.param_request_list_send(s.system, s.component)
            params = {}
            expected = None
            end = time.monotonic() + 45
            while time.monotonic() < end:
                m = s.receive()
                if m is not None and m.get_type() == 'PARAM_VALUE':
                    params[m.param_id] = m.param_value
                    expected = m.param_count
                    if len(params) >= expected:
                        break
            if expected is None or len(params) != expected:
                raise RuntimeError('Lista incompleta: ' + str(len(params)) + '/' + str(expected))
            target.write_text(json.dumps(params, indent=2, sort_keys=True), encoding='utf-8')
            hb = s.wait(lambda x: x.get_type() == 'HEARTBEAT', 5, 'Sem estado final')
            s.record('estado_final', mode=hb.custom_mode, armed=bool(hb.base_mode & 128),
                     parametros_recebidos=len(params), parametros_esperados=expected)
            if hb.base_mode & 128:
                raise RuntimeError('Estado final ainda armado')
            return 0
        finally:
            c.close()


if __name__ == '__main__':
    raise SystemExit(main())
