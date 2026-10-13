"""Leitura do TF-Luna parametrizado no modelo S500 do Gazebo Harmonic.

Uso (no mesmo ambiente do Gazebo):
    python3 python/controle/leitura_infravermelho_s500.py

O sensor gpu_lidar publica gz.msgs.LaserScan, com distâncias em metros.
Alcance, frequência, FoV e montagem estão em gazebo/modelos/quadricoptero/model.sdf.
Referências e instruções: gazebo/README_infravermelho.md.
"""

import argparse
import math
import sys
import threading
import time


TOPICO_PADRAO = "/s500/infravermelho/scan"


def distancia_valida(mensagem):
    """Reduz o FoV à menor distância; None indica medição inválida/sem retorno."""
    minimo, maximo = mensagem.range_min, mensagem.range_max
    if not (math.isfinite(minimo) and math.isfinite(maximo) and 0 < minimo < maximo):
        return None
    if not mensagem.ranges:
        return None
    # -inf ou um raio abaixo do mínimo indica alvo na zona cega.
    # +inf representa ausência de retorno; não deve virar uma distância de 8 m.
    if any(math.isnan(valor) or valor < minimo for valor in mensagem.ranges):
        return None
    distancia = min(mensagem.ranges)
    if not math.isfinite(distancia) or distancia > maximo:
        return None
    return round(distancia, 2)  # Saída de 1 cm, como no TF-Luna.


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", default=TOPICO_PADRAO,
                        help="Tópico Gazebo Transport definido no SDF")
    parser.add_argument("--data-timeout", type=float, default=10.0,
                        help="Tempo máximo sem mensagens, em segundos")
    parser.add_argument("--print-rate", type=float, default=10.0,
                        help="Frequência de exibição em Hz; o sensor continua em 100 Hz")
    args = parser.parse_args()
    if any(not math.isfinite(valor) or valor <= 0
           for valor in (args.data_timeout, args.print_rate)):
        parser.error("--data-timeout e --print-rate devem ser finitos e positivos")

    try:
        from gz.msgs10.laserscan_pb2 import LaserScan
        from gz.transport13 import Node
    except ImportError:
        print("Erro: bindings Python do Gazebo Harmonic indisponíveis. "
              "Use o Python do ambiente com python3-gz-transport13 e python3-gz-msgs10.",
              file=sys.stderr)
        return 1

    # O callback roda em outra thread. Guarda só a mensagem mais recente,
    # evitando acúmulo quando a exibição é mais lenta que o sensor.
    bloqueio = threading.Lock()
    estado = [None, time.monotonic(), 0]

    def receber(mensagem):
        with bloqueio:
            estado[:] = [mensagem, time.monotonic(), estado[2] + 1]

    node = Node()
    if not node.subscribe(LaserScan, args.topic, receber):
        print(f"Erro ao assinar o tópico {args.topic}.", file=sys.stderr)
        return 1

    print(f"Lendo {args.topic}. Inicie a simulação; Ctrl+C para encerrar.", flush=True)
    ultima_sequencia = 0
    try:
        while True:
            with bloqueio:
                mensagem, recebida_em, sequencia = estado
            if time.monotonic() - recebida_em > args.data_timeout:
                print("Erro: tópico sem mensagens. Verifique o mundo, o plugin Sensors "
                      "e se a simulação está em execução.", file=sys.stderr)
                return 1
            if mensagem is not None and sequencia != ultima_sequencia:
                ultima_sequencia = sequencia
                distancia = distancia_valida(mensagem)
                stamp = mensagem.header.stamp
                tempo_simulado = stamp.sec + stamp.nsec / 1e9
                leitura = "sem retorno ou fora da faixa" if distancia is None else f"{distancia:.2f} m"
                print(f"t={tempo_simulado:.3f} s | TF-Luna: {leitura}", flush=True)
            time.sleep(1.0 / args.print_rate)
    except KeyboardInterrupt:
        print("\nLeitura encerrada.")
        return 0


if __name__ == "__main__":
    sys.exit(main())
