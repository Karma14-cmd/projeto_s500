"""Teste de bancada dos seis sensores com o S500 e o cenário do grupo.

Abre o Gazebo sem interface com o modelo estático em duas poses e confere se
IMU, GPS, bússola, barômetro, TF-Luna e câmera publicam leituras coerentes.

Requer Gazebo Harmonic (``gz`` e ``ruby`` no PATH) e os bindings Python
``gz.transport13``/``gz.msgs10``. Exemplo, a partir da raiz do repositório:

    python python/testes/testar_sensores_gazebo.py

Os registros ficam em ``resultados/sensores/<data-hora>/`` (ignorado pelo Git).
"""

import argparse
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import runpy
import shutil
import subprocess
import sys
import tempfile
import threading
import time
import uuid
import xml.etree.ElementTree as ET

MODELO = Path("gazebo/modelos/quadricoptero")
MUNDO = Path("gazebo/mundos/mundo_s500.sdf")
RAIZ_REPO = Path(__file__).resolve().parents[2]


def preparar(checkout, fonte, pose):
    arvore = ET.parse(fonte / MUNDO)
    mundo = arvore.find("world")
    inclui = mundo.find("include")
    configuracao = ET.parse(checkout / MODELO / "model.config")
    caminho_modelo = configuracao.findtext("sdf")
    modelo = ET.parse(fonte / MODELO / caminho_modelo).find("model")
    modelo = copy.deepcopy(modelo)
    for plugin in modelo.findall("plugin"):
        modelo.remove(plugin)
    ET.SubElement(modelo, "static").text = "true"
    ET.SubElement(modelo, "pose").text = pose
    for uri in modelo.findall(".//mesh/uri"):
        caminho = uri.text.removeprefix("model://quadricoptero/")
        malha = checkout / MODELO / caminho
        if not malha.is_file():
            raise FileNotFoundError(malha)
        uri.text = malha.as_posix()
    mundo.remove(inclui)
    mundo.append(modelo)
    return arvore


def executar_pose(comando, arvore, pasta, nome, tipos):
    from gz.transport13 import Node

    with tempfile.TemporaryDirectory(prefix="s500_sensores_", dir=pasta) as temporaria:
        sdf = Path(temporaria) / "bancada.sdf"
        arvore.write(sdf, encoding="utf-8", xml_declaration=True)
        with (pasta / f"{nome}.log").open("wb") as arquivos:
            servidor = subprocess.Popen(
                comando + ["sim", "-s", "-r", "-v", "3", str(sdf)],
                cwd=temporaria, stdout=arquivos, stderr=subprocess.STDOUT,
                creationflags=subprocess.CREATE_NO_WINDOW,
            )
            abertas = []
            node = None
            try:
                time.sleep(.8)
                if servidor.poll() is not None:
                    raise RuntimeError(f"Gazebo saiu com código {servidor.returncode}. Veja {arquivos.name}.")

                node = Node()
                registros = {chave: [] for chave in tipos}
                lock = threading.Lock()

                def receber(chave, msg):
                    with lock:
                        if len(registros[chave]) < 3:
                            registros[chave].append(msg)

                for chave, (tipo, topico) in tipos.items():
                    if not node.subscribe(tipo, topico, lambda msg, id_sensor=chave: receber(id_sensor, msg)):
                        raise RuntimeError(f"Não foi possível assinar {topico}")
                    abertas.append(topico)
                limite = time.monotonic() + 40
                while time.monotonic() < limite:
                    if servidor.poll() is not None:
                        raise RuntimeError(f"Gazebo saiu com código {servidor.returncode}. Veja {arquivos.name}.")
                    with lock:
                        if all(len(amostras) == 3 for amostras in registros.values()):
                            return registros
                    time.sleep(.10)
                faltando = [chave for chave, amostras in registros.items() if len(amostras) < 3]
                raise RuntimeError(f"Sem três amostras de {', '.join(faltando)}. Veja {arquivos.name}.")
            finally:
                if node is not None:
                    for topico in abertas:
                        node.unsubscribe(topico)
                if servidor.poll() is None:
                    servidor.terminate()
                    servidor.wait(timeout=10)


def valores(amostras, distancia_valida):
    imu = amostras["imu"][-1]
    gps = amostras["gps"][-1]
    campo = amostras["bussola"][-1].field_tesla
    frame = amostras["camera"][-1]
    return {
        "aceleracao_m_s2": [getattr(imu.linear_acceleration, eixo) for eixo in "xyz"],
        "orientacao": [getattr(imu.orientation, eixo) for eixo in "wxyz"],
        "imu_tempos": [i.header.stamp.sec + i.header.stamp.nsec / 1e9 for i in amostras["imu"]],
        "gps": [gps.latitude_deg, gps.longitude_deg, gps.altitude],
        "campo_mag": [getattr(campo, eixo) for eixo in "xyz"],
        "pressao_pa": amostras["barometro"][-1].pressure,
        "tf_luna_m": distancia_valida(amostras["tf_luna"][-1]),
        "camera_dimensoes": [frame.width, frame.height],
        "camera_bytes": len(frame.data),
        "camera_sha256": hashlib.sha256(frame.data).hexdigest(),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--checkout", type=Path, default=RAIZ_REPO,
                        help="raiz do repositório com a malha e o model.config (padrão: este repositório)")
    parser.add_argument("--sdf", type=Path, default=None,
                        help="pasta de onde ler model.sdf e mundo_s500.sdf (padrão: --checkout)")
    parser.add_argument("--saida", type=Path, default=None,
                        help="pasta dos registros (padrão: <checkout>/resultados/sensores)")
    parser.add_argument("--validar-sdf", action="store_true",
                        help="apenas verifica se os SDFs gerados carregam, sem simular")
    args = parser.parse_args()
    checkout = args.checkout.resolve()
    fonte = (args.sdf or checkout).resolve()
    raiz_saida = (args.saida or checkout / "resultados" / "sensores").resolve()
    gz = shutil.which("gz")
    ruby = shutil.which("ruby")
    if not gz or not ruby:
        raise RuntimeError("Ative o ambiente Pixi do Gazebo Harmonic.")
    comando = [ruby, str(Path(gz).with_suffix(""))]
    inclinacao = math.radians(10)
    poses = (("pose-a", "0 0 0.6 0 0 0"), ("pose-b", f"2 1 1.8 0 {inclinacao} 0.61"))
    mundos = [(nome, preparar(checkout, fonte, pose)) for nome, pose in poses]

    if args.validar_sdf:
        with tempfile.TemporaryDirectory() as temp:
            for nome, arvore in mundos:
                destino = Path(temp) / f"{nome}.sdf"
                arvore.write(destino, encoding="utf-8", xml_declaration=True)
                subprocess.run(comando + ["sdf", "-k", str(destino)], check=True)
        print("SDF do grupo carregável nas duas poses. Publicação dos sensores depende do ensaio em execução.")
        return 0

    os.environ["GZ_PARTITION"] = "s500_sensores_grupo_" + uuid.uuid4().hex
    os.environ["GALLIUM_DRIVER"] = "llvmpipe"

    from gz.msgs10.fluid_pressure_pb2 import FluidPressure
    from gz.msgs10.image_pb2 import Image
    from gz.msgs10.imu_pb2 import IMU
    from gz.msgs10.laserscan_pb2 import LaserScan
    from gz.msgs10.magnetometer_pb2 import Magnetometer
    from gz.msgs10.navsat_pb2 import NavSat

    tipos = {
        "imu": (IMU, "/imu"),
        "gps": (NavSat, "/s500/gps"),
        "bussola": (Magnetometer, "/s500/bussola"),
        "barometro": (FluidPressure, "/s500/barometro"),
        "tf_luna": (LaserScan, "/s500/infravermelho/scan"),
        "camera": (Image, "/s500/camera"),
    }
    leitor = runpy.run_path(str(checkout / "python/controle/leitura_infravermelho_s500.py"))
    pasta = raiz_saida / time.strftime("%Y%m%d-%H%M%S")
    pasta.mkdir(parents=True)
    saida = {"checkout": str(checkout), "fontes_sdf": str(fonte), "poses": {}}
    for nome, arvore in mundos:
        print(f"Testando {nome} com a malha, o modelo e o mundo do grupo.", flush=True)
        try:
            leituras = executar_pose(comando, arvore, pasta, nome, tipos)
            saida["poses"][nome] = valores(leituras, leitor["distancia_valida"])
        except Exception as erro:
            saida.update(aprovado=False, falha=str(erro))
            (pasta / "resultado.json").write_text(json.dumps(saida, ensure_ascii=False, indent=2), encoding="utf-8")
            print(f"Ensaio interrompido em {nome}. Consulte {pasta}.", file=sys.stderr)
            return 1

    a, b = saida["poses"].values()
    testes = {
        "imu": all(all(math.isfinite(v) for v in dados["aceleracao_m_s2"] + dados["orientacao"])
                   and len(set(dados["imu_tempos"])) == 3 for dados in (a, b)),
        "gps": max(abs(a["gps"][i] - b["gps"][i]) for i in (0, 1)) > 1e-6,
        "bussola": math.dist(a["campo_mag"], b["campo_mag"]) > 1e-6,
        "barometro": a["pressao_pa"] - b["pressao_pa"] > 4,
        "tf_luna": a["tf_luna_m"] is not None and b["tf_luna_m"] is not None
        and abs(a["tf_luna_m"] - 1.0) < .05 and abs(b["tf_luna_m"] - 2.22) < .10,
        "camera": all(dados["camera_dimensoes"] == [640, 480]
                      and dados["camera_bytes"] == 640 * 480 * 3 for dados in (a, b))
        and a["camera_sha256"] != b["camera_sha256"],
    }
    saida["sensores"] = testes
    saida["aprovado"] = all(testes.values())
    destino = pasta / "resultado.json"
    destino.write_text(json.dumps(saida, ensure_ascii=False, indent=2), encoding="utf-8")
    for nome, passou in testes.items():
        print(f"{nome}: {'OK' if passou else 'REVISAR'}")
    print(f"Resultado: {destino}")
    return 0 if saida["aprovado"] else 1


if __name__ == "__main__":
    sys.exit(main())
