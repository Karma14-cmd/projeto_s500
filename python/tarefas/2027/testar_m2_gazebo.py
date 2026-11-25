"""Valida a arena da M2 e executa a rota escolhida no Gazebo + ArduCopter SITL local."""
import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import shutil
import socket
import subprocess
import sys
import tempfile
import time

from m2_rotas import ROTAS
from voar_m2_r2 import main as voar

PASTA = Path(__file__).resolve().parent
RAIZ = PASTA.parents[2]
INICIADOR = RAIZ / "gazebo/windows/iniciar_sitl.py"


def conferir_portas():
    for porta in (9002, 14550):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as udp:
            udp.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
            try:
                udp.bind(("127.0.0.1", porta))
            except OSError as erro:
                raise RuntimeError(f"UDP {porta} indisponível. Confira se há outra sessão SITL aberta.") from erro


def conferir_processos(pasta, nomes):
    import psutil

    dados = json.loads((pasta / "processos.json").read_text(encoding="utf-8"))
    for nome in nomes:
        try:
            registro = dados[nome]
            processo = psutil.Process(registro["pid"])
            if abs(processo.create_time() - registro["criado"]) < .01 and processo.is_running():
                continue
        except (KeyError, psutil.Error):
            pass
        raise RuntimeError(f"O processo {nome} encerrou. Consulte {pasta / (nome + '.log')}")


def listar_rotas():
    print("\nRotas da M2")
    for rota, (zonas, _) in ROTAS.items():
        print(f"  {rota[1:]}: {' → '.join(zonas)}")


def selecionar_rota():
    listar_rotas()
    while True:
        try:
            numero = input("Escolha 1 a 8 para decolar, ou 0 para cancelar: ").strip()
        except EOFError:
            return None
        if numero == "0":
            return None
        rota = f"R{numero}"
        if rota in ROTAS:
            return rota
        print("Digite um número de 1 a 8 ou 0.")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runtime", type=Path, default=Path.home() / "gz-s500")
    parser.add_argument("--sem-janela", action="store_true", help="Roda o servidor sem abrir o Gazebo gráfico ou o painel")
    parser.add_argument("--so-preparar", action="store_true", help="Gera o cenário e verifica o SDF")
    parser.add_argument("--rota", choices=ROTAS, help="Seleciona a rota antecipadamente para executar sem o menu")
    args = parser.parse_args()

    ruby, gz = shutil.which("ruby"), shutil.which("gz")
    if not ruby or not gz:
        parser.error("Execute este script dentro do ambiente Pixi do S500.")

    saida = RAIZ / "resultados/m2-voos"
    saida.mkdir(exist_ok=True, parents=True)
    prefixo = datetime.now().strftime("%Y%m%d-%H%M%S-")
    pasta = Path(tempfile.mkdtemp(prefix=prefixo, dir=saida))
    mundo = pasta / "mundo_m2.sdf"
    particao = f"s500_m2_{pasta.name}"
    os.environ["GZ_PARTITION"] = particao
    print(f"Resultados deste ensaio: {pasta}", flush=True)

    python = [sys.executable, "-B", "-u"]
    iniciar = python + [str(INICIADOR), "--runtime", str(args.runtime.resolve()),
                        "--pasta-sessao", str(pasta), "--particao", particao]
    parametros = ["--mundo", str(mundo)]
    processos_voo = ["gazebo", "ponte", "sitl"]
    nomes = processos_voo.copy()
    if not args.sem_janela:
        parametros += ["--gui", "--camera-gui", "9 9 25 0 1.5707 1.57079633", "--painel-sensores"]
        nomes += ["gui", "painel"]

    try:
        subprocess.run(python + [str(PASTA / "gerar_cenario.py"), "m2", "--barreiras", "0",
                                  "--saida", str(mundo)], check=True)
        subprocess.run([ruby, str(Path(gz).with_suffix("")), "sdf", "-k", str(mundo)], check=True)
        if args.so_preparar:
            listar_rotas()
            return voar(["--rota", args.rota]) if args.rota else 0

        conferir_portas()
        subprocess.run(iniciar + parametros, check=True)
        time.sleep(2)
        conferir_processos(pasta, nomes)
        rota = args.rota or selecionar_rota()
        if rota is None:
            print("Ensaio cancelado antes da decolagem.")
            return 0

        conferir_processos(pasta, processos_voo)
        arquivo = pasta / f"m2_{rota.lower()}_voo.jsonl"
        return voar(["--executar", "--rota", rota, "--log", str(arquivo)])
    except (OSError, RuntimeError, subprocess.CalledProcessError) as erro:
        print(f"Falha no ensaio: {erro}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print("Ensaio interrompido.", file=sys.stderr)
        return 130
    finally:
        if (pasta / "processos.json").is_file():
            subprocess.run(iniciar + ["--parar"], check=False)


if __name__ == "__main__":
    sys.exit(main())
