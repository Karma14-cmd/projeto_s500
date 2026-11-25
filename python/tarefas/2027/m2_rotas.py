"""Rotas, gabarito transcrito e pontuação da M2 no simulador."""
import argparse
from itertools import pairwise
import math
from pathlib import Path

# Regulamento 2027 v1.0, Tabela 2 e custos Cd.
ROTAS = {
    "R1": ("SADG", 77.0), "R2": ("SACDG", 96.8),
    "R3": ("SACEG", 96.8), "R4": ("SBEG", 77.0),
    "R5": ("SBCEG", 96.8), "R6": ("SBCDG", 96.8),
    "R7": ("SCDG", 77.0), "R8": ("SCEG", 77.0),
}

# Vértices lidos na grade da Figura 3, em metros.
ZONAS = {
    "S": ((0, 5), (7, 6), (5, 8), (5, 10), (7, 11.5), (0, 13)),
    "A": ((0, 0), (9, 0), (9, 4), (7, 6), (0, 5)),
    "B": ((0, 13), (7, 11.5), (10.5, 14), (10.5, 18), (0, 18)),
    "C": ((5, 8), (7, 6), (9, 4), (14, 6.5), (13.5, 11.5), (10.5, 14), (7, 11.5), (5, 10)),
    "D": ((9, 0), (18, 0), (18, 5), (14, 6.5), (9, 4)),
    "E": ((10.5, 14), (13.5, 11.5), (18, 13), (18, 18), (10.5, 18)),
    "G": ((14, 6.5), (18, 5), (18, 13), (13.5, 11.5)),
}
GABARITO_TREINO = {"A": 40, "B": -20, "C": 40, "D": 20, "E": -20}

LADO_ARENA = 18
DECOLAGEM = (1, 9)
PONTOS_ZONAS = {
    "S": DECOLAGEM, "A": (6, 3), "B": (6, 15), "C": (9.6, 9),
    "D": (13.4, 3), "E": (14, 15), "G": (16, 9),
}


def pontos_rota(rota):
    return tuple((zona, PONTOS_ZONAS[zona]) for zona in ROTAS[rota][0])


def zona_em(x, y, *, margem=0):
    """Retorna a zona do ponto, com margem opcional para ignorar a fronteira."""
    for zona, vertices in ZONAS.items():
        dentro = False
        perto_da_borda = False
        for (x1, y1), (x2, y2) in pairwise(vertices + vertices[:1]):
            dx, dy = x2 - x1, y2 - y1
            t = max(0, min(1, ((x - x1) * dx + (y - y1) * dy) / (dx * dx + dy * dy)))
            if math.hypot(x - x1 - t * dx, y - y1 - t * dy) <= margem:
                perto_da_borda = True
            if (y1 > y) != (y2 > y) and x < x1 + (y - y1) * dx / dy:
                dentro = not dentro
        if dentro and not perto_da_borda:
            return zona
    return None


def ler_gabarito(texto):
    valores = {}
    for par in texto.split(","):
        zona, valor = par.upper().split("=")
        zona = zona.strip()
        if zona not in "ABCDE" or len(zona) != 1 or zona in valores:
            raise ValueError(f"Zona inválida ou duplicada: {zona}")
        valores[zona] = int(valor)
    if set(valores) != set("ABCDE") or any(v not in (-20, 20, 40) for v in valores.values()):
        raise ValueError("Informe A, B, C, D e E com valores -20, 20 ou 40.")
    return valores


def pontuar(rota, percurso, segundos, altura_maxima, gabarito=None):
    """Pontua a sequência observada até o pouso, sem modelar decisões dos fiscais."""
    sequencia, custo = ROTAS[rota]
    if not math.isfinite(segundos) or segundos < 0:
        raise ValueError("Tempo inválido.")
    if not math.isfinite(altura_maxima) or altura_maxima < 0:
        raise ValueError("Altura inválida.")
    if altura_maxima > 3 or segundos >= 300 or not percurso or not sequencia.startswith(percurso):
        return 0.0

    nivel = int(gabarito is not None)
    recompensa = sum(gabarito[zona] for zona in percurso[1:-1]) if nivel else 0
    completude = (len(percurso) - 1) / (len(sequencia) - 1)
    fator_tempo = 1 - 0.15 * max(0, segundos / 60 - 1)
    pontos = (177 - 30 * nivel + recompensa - custo) * completude * fator_tempo
    return max(0.0, min(150 if nivel else 100, pontos))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gabarito", type=ler_gabarito, help="ex.: A=40,B=-20,C=40,D=20,E=-20")
    parser.add_argument("--rota", choices=ROTAS)
    parser.add_argument("--percurso", help="zonas observadas até o pouso, ex.: SACD")
    parser.add_argument("--segundos", type=float, default=60)
    parser.add_argument("--altura-maxima", type=float, default=3)
    parser.add_argument("--saida", type=Path, help="salva a transcrição e a rota escolhida em TXT")
    args = parser.parse_args()
    if args.percurso and not args.rota:
        parser.error("--percurso exige a rota declarada em --rota.")

    try:
        ranking = sorted(
            ROTAS,
            key=lambda rota: pontuar(rota, ROTAS[rota][0], args.segundos, args.altura_maxima, args.gabarito),
            reverse=True,
        )
        rota = args.rota or ranking[0]
        percurso = args.percurso or ROTAS[rota][0]
        pontos = pontuar(rota, percurso, args.segundos, args.altura_maxima, args.gabarito)
    except ValueError as erro:
        parser.error(str(erro))

    print(f"{rota}, sequência prevista {ROTAS[rota][0]}, registrada {percurso}, {pontos:.2f} pontos")
    if args.saida:
        args.saida.parent.mkdir(parents=True, exist_ok=True)
        linhas = [f"{zona}={valor}" for zona, valor in sorted((args.gabarito or {}).items())]
        linhas.append(f"Rota: {rota} ({' -> '.join(ROTAS[rota][0])})")
        args.saida.write_text("\n".join(linhas) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
