"""Regressão dos cenários gerados e dos exemplos de pontuação da M2."""
from itertools import pairwise
from pathlib import Path
import sys
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest
from unittest.mock import patch

MISSOES = Path(__file__).resolve().parents[1] / "2027"
sys.path.insert(0, str(MISSOES))

from gerar_cenario import montar_m1, montar_m2, montar_mundo
from m2_rotas import GABARITO_TREINO, LADO_ARENA, PONTOS_ZONAS, ROTAS, ZONAS, ler_gabarito, pontuar, pontos_rota, zona_em
from testar_m2_gazebo import selecionar_rota
from voar_m2_r2 import alvo_ned


class Missoes2027Tests(unittest.TestCase):
    def preparar_m2(self, barreiras):
        pasta = self.enterContext(TemporaryDirectory())
        _, mundo, drone = montar_mundo()
        montar_m2(mundo, drone, GABARITO_TREINO, barreiras, Path(pasta))
        return mundo, drone

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
        mundo, drone = self.preparar_m2(4)
        self.assertEqual(drone.findtext("pose").split()[:2], ["1", "9"])
        zonas = [item for item in mundo.findall("model") if item.attrib["name"].startswith("zona_")]
        barreiras = [item for item in mundo.findall("model") if item.attrib["name"].startswith("barreira_")]
        self.assertEqual(len(zonas), 7)
        self.assertEqual(len(barreiras), 4)
        for zona in zonas:
            letra = zona.attrib["name"][-1]
            self.assertIsNotNone(zona.find(f".//visual[@name='rotulo_{letra}_0_0_visual']"))
            self.assertIsNone(zona.find(".//collision"))
        eixos = mundo.find("model[@name='eixos_decolagem']")
        self.assertIsNotNone(eixos.find(".//visual[@name='eixo_X_visual']"))
        self.assertIsNotNone(eixos.find(".//visual[@name='eixo_Y_visual']"))
        for letra in GABARITO_TREINO:
            self.assertIsNotNone(barreiras[0].find(f".//visual[@name='disco_{letra}_visual']"))

    def test_arena_colorida_sem_barreiras(self):
        mundo, _ = self.preparar_m2(0)
        self.assertEqual(mundo.attrib["name"], "mundo_m2_sem_barreiras")
        self.assertFalse(any(m.attrib["name"].startswith("barreira_") for m in mundo.findall("model")))
        cores = set()
        for zona, vertices in ZONAS.items():
            piso = mundo.find(f".//visual[@name='piso_{zona}']")
            cores.add(piso.findtext("material/diffuse"))
            arquivo = Path(piso.findtext("geometry/mesh/uri"))
            pontos = [tuple(map(float, linha.split()[1:3])) for linha in arquivo.read_text().splitlines()
                      if linha.startswith("vertex ")]
            for (centro, a, b), borda in zip(zip(pontos[::3], pontos[1::3], pontos[2::3]),
                                            pairwise(vertices + vertices[:1]), strict=True):
                self.assertEqual(centro, PONTOS_ZONAS[zona])
                self.assertEqual((a, b), borda)
                self.assertGreater((b[0] - a[0]) * (centro[1] - a[1]) - (b[1] - a[1]) * (centro[0] - a[0]), 0)
        self.assertEqual(len(cores), len(ZONAS))

    def test_grade_e_numeracao_usam_os_eixos_da_arena(self):
        mundo, _ = self.preparar_m2(0)
        self.assertEqual(mundo.findtext("scene/grid"), "false")
        grade = mundo.find("model[@name='grade_m2']/link")
        self.assertIsNone(grade.find("collision"))
        for i in range(1, LADO_ARENA):
            for eixo, centro in (("x", (i, LADO_ARENA / 2)), ("y", (LADO_ARENA / 2, i))):
                linha = grade.find(f"visual[@name='grade_{eixo}_{i}_visual']")
                self.assertEqual(tuple(map(float, linha.findtext("pose").split()[:2])), centro)
                self.assertEqual(float(linha.findtext("geometry/box/size").split()[0]), LADO_ARENA)
        self.assertIsNotNone(grade.find("visual[@name='numero_x_18_1_0_0_visual']"))
        self.assertIsNotNone(grade.find("visual[@name='numero_y_18_1_0_0_visual']"))

    def test_trajetorias_retas_cruzam_apenas_as_zonas_previstas(self):
        for rota, (sequencia, _) in ROTAS.items():
            with self.subTest(rota=rota):
                visitadas = []
                for (_, (x1, y1)), (_, (x2, y2)) in pairwise(pontos_rota(rota)):
                    for i in range(201):
                        t = i / 200
                        zona = zona_em(x1 + t * (x2 - x1), y1 + t * (y2 - y1), margem=.20)
                        if zona and (not visitadas or visitadas[-1] != zona):
                            visitadas.append(zona)
                self.assertEqual("".join(visitadas), sequencia)

    def test_menu_rejeita_opcao_invalida_e_aceita_rota_ou_cancelamento(self):
        with patch("builtins.print"), patch("builtins.input", side_effect=["x", "9", " 4 "]):
            self.assertEqual(selecionar_rota(), "R4")
        with patch("builtins.print"), patch("builtins.input", return_value="0"):
            self.assertIsNone(selecionar_rota())

    def test_alvo_ned_usa_a_origem_s_e_inverte_y(self):
        origem = SimpleNamespace(x=20, y=-10)
        self.assertEqual(alvo_ned(origem, (6, 3), -2), (25, -4, -2))
        self.assertEqual(alvo_ned(origem, (16, 9), -2), (35, -10, -2))

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
