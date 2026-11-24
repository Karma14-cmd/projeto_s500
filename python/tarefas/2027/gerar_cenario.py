"""Gera cenários de treino M1 e M2 a partir do S500 do repositório."""
import argparse
from itertools import pairwise
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

from m2_rotas import GABARITO_TREINO, ZONAS, ler_gabarito

RAIZ = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(RAIZ / "gazebo/windows"))
from gerar_mundo_voo import montar_mundo

PRETO = "0.03 0.03 0.03 1"
BRANCO = "0.95 0.95 0.95 1"
AZUL = "0.05 0.12 0.85 1"
CINZA = "0.32 0.32 0.32 1"
VERMELHO = "0.85 0.04 0.04 1"
LARANJA = "1 0.36 0.03 1"
LETRAS = {
    "A": (((-1, -1), (0, 1), (1, -1)), ((-.5, 0), (.5, 0))),
    "B": (((-1, -1), (-1, 1), (.5, 1), (1, .5), (.5, 0), (-1, 0)),
          ((.5, 0), (1, -.5), (.5, -1), (-1, -1))),
    "C": (((1, 1), (-.5, 1), (-1, .5), (-1, -.5), (-.5, -1), (1, -1)),),
    "D": (((-1, -1), (-1, 1), (.3, 1), (1, .4), (1, -.4), (.3, -1), (-1, -1)),),
    "E": (((1, -1), (-1, -1), (-1, 1), (1, 1)), ((-1, 0), (.65, 0))),
}


def elemento(pai, nome, texto, **atributos):
    no = ET.SubElement(pai, nome, atributos)
    no.text = texto
    return no


def modelo(mundo, nome, posicao, *, estatico):
    item = ET.SubElement(mundo, "model", name=nome)
    elemento(item, "pose", posicao)
    elemento(item, "static", "true" if estatico else "false")
    return ET.SubElement(item, "link", name="corpo")


def forma(link, nome, posicao, tipo, medidas, cor, *, colisao=False):
    classes = ("visual", "collision") if colisao else ("visual",)
    for classe in classes:
        item = ET.SubElement(link, classe, name=f"{nome}_{classe}")
        elemento(item, "pose", posicao)
        geometria = ET.SubElement(ET.SubElement(item, "geometry"), tipo)
        for medida, valor in medidas.items():
            elemento(geometria, medida, str(valor))
        if classe == "visual":
            material = ET.SubElement(item, "material")
            elemento(material, "ambient", cor)
            elemento(material, "diffuse", cor)


def poligono(lados, raio):
    return [(raio * math.cos(a), raio * math.sin(a))
            for a in (math.pi / 2 + i * math.tau / lados for i in range(lados))]


def traco(link, nome, inicio, fim, *, largura=.025, plano="xy", cor=PRETO, colisao=False):
    a, b, c = [(x + y) / 2 for x, y in zip(inicio, fim)]
    dx, dy, dz = [y - x for x, y in zip(inicio, fim)]
    extensao = math.dist(inicio, fim)
    if plano == "xy":
        posicao = f"{a} {b} {c} 0 0 {math.atan2(dy, dx)}"
        tamanho = f"{extensao} {largura} {largura}"
    elif plano == "yz":
        posicao = f"{a} {b} {c} {math.atan2(dz, dy)} 0 0"
        tamanho = f"{largura} {extensao} {largura}"
    else:
        posicao = f"{a} {b} {c} 0 {math.atan2(-dz, dx)} 0"
        tamanho = f"{extensao + largura} {largura} {largura}"
    forma(link, nome, posicao, "box", {"size": tamanho}, cor, colisao=colisao)


def base(mundo, nome, x, y, lados=None):
    link = modelo(mundo, nome, f"{x} {y} 0.006 0 0 0", estatico=True)
    forma(link, "disco", "0 0 0 0 0 0", "cylinder",
          {"radius": .4, "length": .012}, AZUL if lados is None else BRANCO)
    if lados is None:
        for i, (cx, cy) in enumerate(poligono(4, .36)):
            traco(link, f"marca_{i}", (.87 * cx, .87 * cy, .012), (cx, cy, .012))
        return
    vertices = poligono(lados, .35)
    if lados == 10:
        vertices = [(x * .46, y * .46) if i % 2 else (x, y)
                    for i, (x, y) in enumerate(vertices)]
    for i, ((x1, y1), (x2, y2)) in enumerate(pairwise(vertices + vertices[:1])):
        traco(link, f"borda_{i}", (x1, y1, .012), (x2, y2, .012))


def montar_m1(mundo, drone, alvo):
    base(mundo, "decolagem", 0, 0)
    for nome, (x, y, lados) in {
        "hexagono": (-1.7, -1.7, 6), "estrela": (-1.7, 1.7, 10),
        "triangulo": (1.7, 1.7, 3),
    }.items():
        base(mundo, nome, x, y, lados)

    for nome, x in (("esquerda", -3.0), ("direita", 3.0)):
        link = modelo(mundo, f"alvo_{nome}", f"{x} 0 0 0 0 0", estatico=True)
        forma(link, "mangueira", f"0 0 1.7 {math.pi / 2} 0 0", "cylinder",
              {"radius": .00635, "length": 4}, VERMELHO, colisao=True)
        for i, y in enumerate((-2, 2)):
            forma(link, f"poste_{i}", f"0 {y} .9 0 0 0", "box",
                  {"size": ".08 .08 1.8"}, CINZA, colisao=True)
        if nome == alvo:
            forma(link, "esfera", "0 1.2 1.825 0 0 0", "sphere", {"radius": .125}, LARANJA)

    gancho = modelo(mundo, "gancho", "0 0 .112 0 0 0", estatico=False)
    inercia = ET.SubElement(gancho, "inertial")
    elemento(inercia, "mass", "0.035")
    tensor = ET.SubElement(inercia, "inertia")
    for eixo in ("ixx", "iyy", "izz"):
        elemento(tensor, eixo, "0.00008")
    arco = [math.radians(graus) for graus in range(-40, 221, 20)]
    pontos = [(.054 * math.cos(a), 0, .07 * math.sin(a)) for a in arco]
    for i, (inicio, fim) in enumerate(pairwise(pontos)):
        traco(gancho, f"segmento_{i}", inicio, fim, largura=.008,
              plano="xz", cor=LARANJA, colisao=True)

    junta = ET.SubElement(drone, "plugin", {
        "filename": "gz-sim-detachable-joint-system",
        "name": "gz::sim::systems::DetachableJoint",
    })
    for campo, valor in {
        "parent_link": "base_link", "child_model": "gancho", "child_link": "corpo",
        "detach_topic": "/s500/gancho/soltar", "output_topic": "/s500/gancho/estado",
    }.items():
        elemento(junta, campo, valor)


def desenhar_gabarito(link, valores):
    for i, (letra, valor) in enumerate(sorted(valores.items())):
        y = -.65 + i * .325
        z = .45 if i % 2 == 0 else .73
        forma(link, f"disco_{letra}", f"-.052 {y} {z} 0 {math.pi / 2} 0", "cylinder",
              {"radius": .10, "length": .004}, BRANCO)
        vertices = poligono({40: 4, 20: 5, -20: 24}[valor], .076)
        for j, ((y1, z1), (y2, z2)) in enumerate(pairwise(vertices + vertices[:1])):
            traco(link, f"forma_{letra}_{j}", (-.058, y + y1, z + z1), (-.058, y + y2, z + z2),
                  largura=.006, plano="yz")
        for ramo, pontos in enumerate(LETRAS[letra]):
            for j, ((y1, z1), (y2, z2)) in enumerate(pairwise(pontos)):
                traco(link, f"letra_{letra}_{ramo}_{j}",
                      (-.061, y + .028 * y1, z + .036 * z1),
                      (-.061, y + .028 * y2, z + .036 * z2), largura=.007, plano="yz")


def montar_m2(mundo, drone, valores, quantidade):
    drone.find("pose").text = "1 9 .31 0 0 0"
    base(mundo, "decolagem_s", 1, 9)
    segmentos = set()
    for zona, vertices in ZONAS.items():
        link = modelo(mundo, f"zona_{zona}", "0 0 0 0 0 0", estatico=True)
        for i, ((x1, y1), (x2, y2)) in enumerate(pairwise(vertices + vertices[:1])):
            trecho = frozenset(((x1, y1), (x2, y2)))
            if trecho not in segmentos:
                traco(link, f"fronteira_{i}", (x1, y1, .012), (x2, y2, .012), cor=BRANCO)
                segmentos.add(trecho)

    fronteiras = (((5, 8), (5, 10)), ((7, 6), (9, 4)),
                   ((10.5, 14), (13.5, 11.5)), ((14, 6.5), (18, 5)))
    for i, ((x1, y1), (x2, y2)) in enumerate(fronteiras[:quantidade]):
        x, y = (x1 + x2) / 2, (y1 + y2) / 2
        angulo = math.atan2(y2 - y1, x2 - x1) - math.pi / 2
        link = modelo(mundo, f"barreira_{i + 1}", f"{x} {y} 1.5 0 0 {angulo}", estatico=True)
        forma(link, "painel", "0 0 0 0 0 0", "box", {"size": ".10 2 3"}, CINZA, colisao=True)
        if i == 0:
            desenhar_gabarito(link, valores)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("missao", choices=("m1", "m2"))
    parser.add_argument("--saida", type=Path, required=True)
    parser.add_argument("--alvo", choices=("esquerda", "direita"), default="direita")
    parser.add_argument("--gabarito", type=ler_gabarito, default=GABARITO_TREINO)
    parser.add_argument("--barreiras", type=int, choices=range(1, 5), default=2)
    args = parser.parse_args()

    sdf, mundo, drone = montar_mundo()
    if args.missao == "m1":
        montar_m1(mundo, drone, args.alvo)
    else:
        montar_m2(mundo, drone, args.gabarito, args.barreiras)

    args.saida.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(sdf, space="  ")
    sdf.write(args.saida, encoding="utf-8", xml_declaration=True)
    print(f"Cenário gerado em {args.saida.resolve()}")


if __name__ == "__main__":
    main()
