"""Percorre uma rota no mundo M2 sem barreiras do Gazebo + ArduCopter SITL."""
import argparse
import math
from pathlib import Path
import sys
import time

from m2_rotas import DECOLAGEM, LADO_ARENA, ROTAS, pontos_rota, zona_em

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "testes/sitl"))
from voo_autonomo import Session

ALTURA = 2.0
TETO = 3.0
MODO_GUIDED = 4
MODO_LAND = 9
MUNDO = "mundo_m2_sem_barreiras"
ODOMETRIA = "/model/quadricoptero/odometry"


def alvo_ned(origem, ponto, down):
    """Aplica os mesmos sinais X e Y que a ponte JSON do S500."""
    x, y = ponto
    return origem.x + x - DECOLAGEM[0], origem.y - (y - DECOLAGEM[1]), down


def voar_rota(sessao, posicao_gazebo, rota, prazo=240):
    mav = sessao.m
    sequencia = ROTAS[rota][0]
    ocupadas = ["S"]
    altura_maxima = 0.0
    limite = None
    origem_z = 0.0

    def heartbeat(mensagem):
        return mensagem.get_type() == "HEARTBEAT"

    def desarmado(mensagem):
        return heartbeat(mensagem) and not mensagem.base_mode & mav.MAV_MODE_FLAG_SAFETY_ARMED

    def conferir_posicao():
        nonlocal altura_maxima
        if limite and time.monotonic() > limite:
            raise TimeoutError("Tempo de navegação excedido.")
        x, y, z = posicao_gazebo()
        altura_maxima = max(altura_maxima, z)
        if z > TETO:
            raise RuntimeError(f"O S500 ultrapassou {TETO:g} m.")
        if not (0 <= x <= LADO_ARENA and 0 <= y <= LADO_ARENA):
            raise RuntimeError("O S500 saiu da arena.")
        zona = zona_em(x, y, margem=.20)
        if zona and zona != ocupadas[-1]:
            ocupadas.append(zona)
            sessao.record("zona", zona=zona, x=x, y=y, z=z)
            if not sequencia.startswith("".join(ocupadas)):
                raise RuntimeError(f"Desvio da {rota}: {' -> '.join(ocupadas)}")
        return x, y, z

    def confirmar_alvo(ponto):
        x, y, z = conferir_posicao()
        return math.dist((x, y), ponto) <= .35 and abs(z - origem_z - ALTURA) <= .25

    hb = sessao.wait(heartbeat, 45, "Sem heartbeat do SITL")
    if not desarmado(hb):
        raise RuntimeError("O voo deve começar com o S500 desarmado.")
    if sessao.param("SIM_SPEEDUP") <= 0 or sessao.param("SIM_RC_FAIL") != 0:
        raise RuntimeError("Verifique a configuração do SITL e o failsafe RC.")
    sessao.command(mav.MAV_CMD_SET_MESSAGE_INTERVAL, mav.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 100000)
    sessao.command(mav.MAV_CMD_SET_MESSAGE_INTERVAL, mav.MAVLINK_MSG_ID_EKF_STATUS_REPORT, 500000)
    flags_posicao = mav.EKF_ATTITUDE | mav.EKF_VELOCITY_HORIZ | mav.EKF_POS_HORIZ_ABS
    sessao.wait(lambda m: m.get_type() == "EKF_STATUS_REPORT"
                and m.flags & flags_posicao == flags_posicao, 120, "Posição do EKF indisponível.")
    sessao.wait(lambda m: m.get_type() == "LOCAL_POSITION_NED", 30, "Sem posição local.")
    sessao.mode(MODO_GUIDED)
    origem = sessao.wait(lambda m: m.get_type() == "LOCAL_POSITION_NED", 5, "Sem posição inicial.")
    x, y, origem_z = posicao_gazebo()
    if math.dist((x, y), DECOLAGEM) > .35 or origem_z > .45:
        raise RuntimeError("O S500 precisa estar em repouso na base S (1, 9).")
    sessao.record("inicio", rota=rota, x=x, y=y, z=origem_z, altura=ALTURA)
    try:
        sessao.guard = True
        inicio = time.monotonic()
        limite = inicio + prazo
        sessao.command(mav.MAV_CMD_COMPONENT_ARM_DISARM, 1)
        sessao.wait(lambda m: heartbeat(m) and not desarmado(m),
                    10, "Armamento não confirmado.")
        sessao.command(mav.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, ALTURA)
        sessao.wait(lambda m: confirmar_alvo(DECOLAGEM), 40, "Decolagem não confirmada.")
        for zona, ponto in pontos_rota(rota)[1:]:
            alvo = alvo_ned(origem, ponto, origem.z - ALTURA)
            sessao.c.mav.set_position_target_local_ned_send(
                0, sessao.system, sessao.component, mav.MAV_FRAME_LOCAL_NED, 3576,
                *alvo, 0, 0, 0, 0, 0, 0, 0, 0,
            )
            sessao.record("alvo", zona=zona, ponto=ponto)
            sessao.wait(lambda m: confirmar_alvo(ponto), 35, f"Alvo {zona} não atingido.")
            sessao.record("alvo_confirmado", zona=zona)

        sessao.guard = False
        sessao.mode(MODO_LAND)

        def pousou(mensagem):
            conferir_posicao()
            return desarmado(mensagem)

        sessao.wait(pousou, 50, "Desarmamento após o pouso não confirmado.")
        x, y, _ = conferir_posicao()
        if zona_em(x, y, margem=.20) != "G" or "".join(ocupadas) != sequencia:
            raise RuntimeError(f"O pouso ou a sequência de zonas divergiu da {rota}.")
        sessao.record("concluido", rota=rota, percurso=ocupadas, altura_maxima=altura_maxima,
                      pouso=[x, y], duracao_real_s=round(time.monotonic() - inicio, 2))
    except (Exception, KeyboardInterrupt):
        sessao.guard = False
        try:
            hb = sessao.wait(heartbeat, 3, "Sem heartbeat para pouso de recuperação.")
            if not desarmado(hb) and hb.custom_mode == MODO_GUIDED:
                sessao.mode(MODO_LAND)
                sessao.wait(desarmado, 50, "Pouso de recuperação não confirmado.")
                sessao.record("pouso_de_recuperacao")
        except Exception as erro:
            sessao.record("recuperacao_nao_confirmada", erro=str(erro))
        raise


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--executar", action="store_true", help="autoriza o voo no SITL local")
    parser.add_argument("--rota", choices=ROTAS, default="R2")
    parser.add_argument("--log", type=Path)
    args = parser.parse_args(argv)

    print(f"{args.rota} - " + " -> ".join(f"{zona} {ponto}" for zona, ponto in pontos_rota(args.rota)))
    if not args.executar:
        return 0

    from gz.msgs10.odometry_pb2 import Odometry
    from gz.msgs10.world_stats_pb2 import WorldStatistics
    from gz.transport13 import Node
    from pymavlink import mavutil

    node = Node()
    leitura = None
    ultimo_mundo = None

    def receber_odom(mensagem):
        nonlocal leitura
        p = mensagem.pose.position
        leitura = (time.monotonic(), (p.x, p.y, p.z))

    def receber_mundo(mensagem):
        nonlocal ultimo_mundo
        if not mensagem.paused:
            ultimo_mundo = time.monotonic()

    def posicao_gazebo():
        atual = leitura
        if atual is None or time.monotonic() - atual[0] > 2:
            raise TimeoutError("Odometria do S500 ausente ou desatualizada.")
        if ultimo_mundo is None or time.monotonic() - ultimo_mundo > 3:
            raise TimeoutError("O mundo da M2 sem barreiras não está ativo.")
        return atual[1]

    if not node.subscribe(WorldStatistics, f"/world/{MUNDO}/stats", receber_mundo):
        parser.error("Falha ao assinar o estado da simulação.")
    if not node.subscribe(Odometry, ODOMETRIA, receber_odom):
        parser.error("Falha ao assinar a odometria.")
    limite = time.monotonic() + 45
    while time.monotonic() < limite and (leitura is None or ultimo_mundo is None):
        time.sleep(.1)
    try:
        posicao_gazebo()
    except TimeoutError as erro:
        parser.error(str(erro))

    arquivo = args.log or Path(f"resultados/m2_{args.rota.lower()}_voo.jsonl")
    arquivo.parent.mkdir(parents=True, exist_ok=True)
    with arquivo.open("a", encoding="utf-8") as log:
        conexao = mavutil.mavlink_connection(
            "udpin:127.0.0.1:14550", source_system=255, source_component=190,
        )
        sessao = Session(conexao, mavutil.mavlink, log)
        try:
            voar_rota(sessao, posicao_gazebo, args.rota)
        except (Exception, KeyboardInterrupt) as erro:
            sessao.record("aborto", erro=str(erro))
            return 1
        finally:
            conexao.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
