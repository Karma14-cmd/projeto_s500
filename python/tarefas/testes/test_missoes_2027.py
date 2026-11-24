"""Regressão dos cenários gerados e dos exemplos de pontuação da M2."""
from itertools import pairwise
from pathlib import Path
import sys
import unittest

MISSOES = Path(__file__).resolve().parents[1] / "2027"
sys.path.insert(0, str(MISSOES))

from gerar_cenario import montar_m1, montar_m2, montar_mundo
from m2_rotas import GABARITO_TREINO, ROTAS, ZONAS, ler_gabarito, pontuar


class Missoes2027Tests(unittest.TestCase):
    def test_m1_usa_junta_soltar_e_alvo_selecionavel(self):
        _, mundo, drone = montar_mundo()
        montar_m1(mundo, drone, "esquerda")
        self.assertIsNotNone(mundo.find("model[@name='gancho']/link/collision"))
        self.assertIsNotNone(mundo.find("model[@name='alvo_esquerda']//sphere"))
        self.assertIsNone(mundo.find("model[@name='alvo_direita']//sphere"))
        junta = drone.find("plugin[@name='gz::sim::systems::DetachableJoint']")
        self.assertEqual(junta.findtext("child_model"), "gancho")
        self.assertEqual(junta.findtext("detach_topic"), "/s500/gancho/soltar")

    def test_m2_pista_completa_e_gabarito_no_primeiro_painel(self):
        _, mundo, drone = montar_mundo()
        montar_m2(mundo, drone, GABARITO_TREINO, 4)
        self.assertEqual(drone.findtext("pose").split()[:2], ["1", "9"])
        zonas = [item for item in mundo.findall("model") if item.attrib["name"].startswith("zona_")]
        barreiras = [item for item in mundo.findall("model") if item.attrib["name"].startswith("barreira_")]
        self.assertEqual(len(zonas), 7)
        self.assertEqual(len(barreiras), 4)
        for letra in GABARITO_TREINO:
            self.assertIsNotNone(barreiras[0].find(f".//visual[@name='disco_{letra}_visual']"))

    def test_zonas_cobrem_area_de_18_por_18(self):
        area = sum(abs(sum(x1 * y2 - y1 * x2 for (x1, y1), (x2, y2) in
                           pairwise(vertices + vertices[:1]))) / 2 for vertices in ZONAS.values())
        self.assertEqual(area, 18 * 18)

    def test_exemplo_tabela_3_nivel_2(self):
        for percurso, esperado in (("SA", 12.55), ("SAC", 45.10), ("SACD", 97.65), ("SACDG", 150)):
            with self.subTest(percurso=percurso):
                pontos = pontuar("R2", percurso, 50, 2.5, GABARITO_TREINO)
                self.assertAlmostEqual(pontos, esperado, places=2)

    def test_nivel_1_e_limites(self):
        self.assertEqual(pontuar("R1", ROTAS["R1"][0], 60, 3), 100)
        self.assertEqual(pontuar("R1", "SBEG", 50, 3), 0)
        self.assertEqual(pontuar("R1", "SADG", 50, 3.001), 0)
        self.assertEqual(pontuar("R1", "SADG", 300, 2), 0)
        self.assertAlmostEqual(pontuar("R1", "SADG", 120, 2), 85)

    def test_gabarito_valida_zonas_e_valores(self):
        self.assertEqual(ler_gabarito("A=40,B=-20,C=40,D=20,E=-20"), GABARITO_TREINO)
        for texto in ("A=40", "A=20,A=40,B=-20,C=40,D=20,E=-20", "A=10,B=-20,C=40,D=20,E=-20"):
            with self.subTest(texto=texto), self.assertRaises(ValueError):
                ler_gabarito(texto)

    def test_nomes_dos_elementos_do_trem_de_pouso_sao_distintos(self):
        _, _, drone = montar_mundo()
        base = drone.find("link[@name='base_link']")
        nomes = [item.attrib["name"] for item in base if "name" in item.attrib]
        self.assertCountEqual(nomes, set(nomes))


if __name__ == "__main__":
    unittest.main(verbosity=2)
