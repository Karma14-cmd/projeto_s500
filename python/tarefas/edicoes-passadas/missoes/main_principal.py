"""
================================================================================
PROJETO: Sistema de Visão Computacional Unificado - Equipe Eletroquad SAE
SETOR: Controle e Navegação
ARQUIVO: main_principal.py (Código Principal Completo e Independente)

RESUMO DAS FUNCIONALIDADES PARA O TIME DE CONTROLE:
  1. MISSÃO 1 (ArUco 5x5):
     - Detecta o marcador sem inverter os bits internos e espelha a tela.
     - Regra de divisibilidade por 3, 4 ou 5.
     - Modo de Certeza (0% a 100%): Trava no ID confirmado e segue o marcador
       mesmo sob borrão de movimento (motion blur).
  2. MISSÃO 2 (Esfera Laranja 25cm):
     - Filtro HSV + Morfologia + Validação de circularidade.
     - Estimativa de distância real em metros (baseada no diâmetro de 0.25m).
     - Modo de Certeza (0% a 100%): Trava na esfera e acompanha na tela.
  3. MISSÃO 3 (Manômetro Analógico):
     - Detecção por alto contraste de contorno (ignora rostos e fundo).
     - Limiar adaptativo automático para leitura da agulha + média vetorial.
     - Modo de Certeza (0% a 100%): Trava no mostrador e estabiliza o ângulo.

ATALHOS DE OPERAÇÃO EM TEMPO REAL:
  [1] -> Missão 1 (ArUco 5x5)
  [2] -> Missão 2 (Esfera Laranja)
  [3] -> Missão 3 (Manômetro Analógico)
  [r] -> Reseta/Destrava o alvo da missão atual manualmente
  [e] -> Alterna Modo Espelho (ON para bancada / OFF para câmera no drone)
  [q] -> Fecha o programa com segurança

FONTE DE VÍDEO:
  python main_principal.py                 -> webcam (padrão)
  python main_principal.py --fonte gazebo  -> câmera frontal do S500 simulado
                                              (tópico /s500/camera)
================================================================================
"""

import time
import cv2
import numpy as np

import missao2_esfera
import missao3_manometro
from fonte_video import abrir_fonte

# ==============================================================================
# CONFIGURAÇÕES E CONSTANTES GLOBAIS
# ==============================================================================
# Inicialização do detector ArUco 5x5 (IDs de 0 a 999)
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_1000)
aruco_params = cv2.aruco.DetectorParameters()
detector_aruco = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

# ==============================================================================
# VARIÁVEIS DE ESTADO (MODO DE CERTEZA E RASTREAMENTO DAS 3 MISSÕES)
# ==============================================================================
# Estado Missão 1 (ArUco)
m1_certeza = 0
m1_travado = False
m1_id_travado = None
m1_pos_travada = None  # (cx, cy, raio_aprox)
m1_frames_perdidos = 0



def resetar_todas_travas():
    """Reseta o Modo de Certeza e destrava os alvos das Missões 1, 2 e 3."""
    global m1_certeza, m1_travado, m1_id_travado, m1_pos_travada, m1_frames_perdidos

    # Reset Missão 1
    m1_certeza = 0
    m1_travado = False
    m1_id_travado = None
    m1_pos_travada = None
    m1_frames_perdidos = 0

    missao2_esfera.resetar_trava()
    missao3_manometro.resetar_trava()


# ==============================================================================
# MISSÃO 1: ARUCO 5X5 (COM TRAVA DE ALVO E DIVISIBILIDADE)
# ==============================================================================
def processar_missao_1(frame, espelhar=True):
    global m1_certeza, m1_travado, m1_id_travado, m1_pos_travada, m1_frames_perdidos

    h, w = frame.shape[:2]
    centro_cam_x, centro_cam_y = w // 2, h // 2

    # 1. Detecta o ArUco na imagem original (para não inverter a matriz binária)
    gray_raw = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = detector_aruco.detectMarkers(gray_raw)

    # 2. Espelha a imagem para o modo habitual se estiver na bancada
    if espelhar:
        frame = cv2.flip(frame, 1)
        gray = cv2.flip(gray_raw, 1)
    else:
        gray = gray_raw

    marcadores = []
    if ids is not None:
        for i, m_id in enumerate(ids.flatten()):
            id_num = int(m_id)
            pts = corners[i][0].copy()
            if espelhar:
                pts[:, 0] = (w - 1) - pts[:, 0]

            cx = int(np.mean(pts[:, 0]))
            cy = int(np.mean(pts[:, 1]))
            r_aprox = int(
                np.hypot(pts[0][0] - pts[2][0], pts[0][1] - pts[2][1]) / 2.0
            )
            marcadores.append((id_num, cx, cy, max(20, r_aprox), pts))

    pts_desenho = None

    # 3. Máquina de Estados: Confirmação vs. Trava no Alvo
    if not m1_travado:
        if marcadores:
            melhor = max(marcadores, key=lambda m: m[3])
            id_num, cx, cy, r_aprox, pts = melhor

            if m1_id_travado is not None and m1_id_travado != id_num:
                m1_certeza = 10
            else:
                m1_certeza = min(100, m1_certeza + 10)

            m1_id_travado = id_num
            m1_pos_travada = (cx, cy, r_aprox)
            pts_desenho = pts

            if m1_certeza >= 100:
                m1_travado = True
                m1_frames_perdidos = 0
        else:
            m1_certeza = max(0, m1_certeza - 5)
            if m1_certeza == 0:
                m1_id_travado = None
                m1_pos_travada = None
    else:
        lx, ly, lr = m1_pos_travada
        atualizado = False

        for id_num, cx, cy, r_aprox, pts in marcadores:
            if id_num == m1_id_travado:
                novo_x = int(0.20 * lx + 0.80 * cx)
                novo_y = int(0.20 * ly + 0.80 * cy)
                novo_r = int(0.30 * lr + 0.70 * r_aprox)
                m1_pos_travada = (novo_x, novo_y, novo_r)
                pts_desenho = pts
                m1_frames_perdidos = 0
                atualizado = True
                break

        # Rastreador auxiliar de contorno caso o movimento rápido borre os bits do ArUco
        if not atualizado:
            blur = cv2.GaussianBlur(gray, (5, 5), 0)
            _, thresh_escuro = cv2.threshold(blur, 100, 255, cv2.THRESH_BINARY_INV)
            contours, _ = cv2.findContours(
                thresh_escuro, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )

            melhor_cnt = None
            menor_dist = float("inf")

            for cnt in contours:
                if cv2.contourArea(cnt) > 300:
                    (nx, ny), n_raio = cv2.minEnclosingCircle(cnt)
                    dist = np.hypot(nx - lx, ny - ly)
                    if dist < (lr * 2.5) and (lr * 0.4 < n_raio < lr * 2.2):
                        if dist < menor_dist:
                            menor_dist = dist
                            melhor_cnt = (int(nx), int(ny), int(n_raio))

            if melhor_cnt is not None:
                nx, ny, nr = melhor_cnt
                m1_pos_travada = (
                    int(0.30 * lx + 0.70 * nx),
                    int(0.30 * ly + 0.70 * ny),
                    int(0.40 * lr + 0.60 * nr),
                )
                m1_frames_perdidos += 1
            else:
                m1_frames_perdidos += 1

            if m1_frames_perdidos > 60:
                resetar_todas_travas()

    # 4. Desenho de Telemetria na Tela
    if m1_pos_travada is not None and m1_id_travado is not None:
        cx, cy, r_aprox = m1_pos_travada
        desvio_x = cx - centro_cam_x
        desvio_y = cy - centro_cam_y

        divs = [d for d in (3, 4, 5) if m1_id_travado % d == 0]
        txt_divs = str(divs) if divs else "Nenhum (3,4,5)"
        cor = (0, 255, 0) if m1_travado else (0, 165, 255)

        if m1_travado:
            status = (
                "ARUCO TRAVADO (100%)"
                if m1_frames_perdidos == 0
                else "RASTREANDO ARUCO (100%)"
            )
            cv2.rectangle(
                frame,
                (cx - r_aprox - 8, cy - r_aprox - 8),
                (cx + r_aprox + 8, cy + r_aprox + 8),
                (0, 255, 0),
                2,
            )
        else:
            status = f"Confirmando ID {m1_id_travado}... {m1_certeza}%"

        if pts_desenho is not None:
            pts_int = pts_desenho.astype(np.int32).reshape((-1, 1, 2))
            cv2.polylines(frame, [pts_int], isClosed=True, color=cor, thickness=2)
        else:
            cv2.circle(frame, (cx, cy), r_aprox, cor, 2)

        cv2.circle(frame, (cx, cy), 5, (0, 255, 0), -1)
        cv2.line(frame, (centro_cam_x, centro_cam_y), (cx, cy), (0, 255, 255), 2)

        cv2.putText(
            frame,
            f"{status} | ID: {m1_id_travado} | Div: {txt_divs}",
            (max(10, cx - 140), max(25, cy - r_aprox - 15)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.52,
            cor,
            2,
        )
        cv2.putText(
            frame,
            f"Desvio X: {desvio_x}px Y: {desvio_y}px",
            (max(10, cx - 80), min(h - 10, cy + r_aprox + 25)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 255),
            2,
        )
    else:
        cv2.putText(
            frame,
            "Procurando ArUco 5x5...",
            (10, 65),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 0, 255),
            2,
        )

    return frame



def processar_missao_2(frame, espelhar=True):
    return missao2_esfera.processar(
        frame, espelhar, ao_perder=resetar_todas_travas,
        rotulos_trava=("ESFERA TRAVADA (100%)", "MEMORIA ESFERA (100%)"),
    )


def processar_missao_3(frame, espelhar=True):
    return missao3_manometro.processar(frame, espelhar, ao_perder=resetar_todas_travas)


def main():
    import argparse

    parser = argparse.ArgumentParser(description="Visão das missões do S500.")
    parser.add_argument("--fonte", choices=("webcam", "gazebo"), default="webcam",
                        help="webcam (padrão) ou câmera simulada do Gazebo")
    parser.add_argument("--topico", default="/s500/camera",
                        help="tópico gz da câmera quando --fonte gazebo")
    args = parser.parse_args()

    cap = abrir_fonte(args.fonte, args.topico, resolucao=(640, 480))

    if not cap.isOpened():
        print("[ERRO CRÍTICO] Não foi possível acessar a câmera.")
        return

    missao_atual = 1
    # A câmera do drone não precisa de espelho; a webcam de bancada sim.
    modo_espelho = args.fonte == "webcam"
    tempo_anterior = time.time()

    print("=================================================================")
    print(" SISTEMA DE VISÃO UNIFICADO - ELETROQUAD SAE (CONTROLE/NAVEGAÇÃO)")
    print(" Teclas: [1, 2, 3] Missões | [r] Destravar | [e] Espelho | [q] Sair")
    print("=================================================================")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("[AVISO] Falha na captura do frame.")
            break

        if missao_atual == 1:
            frame = processar_missao_1(frame, espelhar=modo_espelho)
            nome_missao = "MISSAO 1: ArUco 5x5"
        elif missao_atual == 2:
            frame = processar_missao_2(frame, espelhar=modo_espelho)
            nome_missao = "MISSAO 2: Esfera Laranja"
        elif missao_atual == 3:
            frame = processar_missao_3(frame, espelhar=modo_espelho)
            nome_missao = "MISSAO 3: Manometro"

        # Cálculo de FPS e HUD Central
        tempo_atual = time.time()
        fps = 1.0 / max(1e-5, (tempo_atual - tempo_anterior))
        tempo_anterior = tempo_atual

        h, w = frame.shape[:2]
        cv2.drawMarker(
            frame, (w // 2, h // 2), (255, 255, 255), cv2.MARKER_CROSS, 20, 1
        )
        cv2.putText(
            frame,
            f"{nome_missao} | FPS: {fps:.0f}",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (255, 255, 0),
            2,
        )

        status_espelho = "ON (Bancada)" if modo_espelho else "OFF (Drone)"
        rodape = f"[1,2,3] Missao | [r] Reset Trava | [e] Espelho: {status_espelho} | [q] Sair"
        cv2.putText(
            frame,
            rodape,
            (10, h - 12),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (200, 200, 200),
            1,
        )

        cv2.imshow("Visao Computacional - Eletroquad SAE", frame)

        tecla = cv2.waitKey(1) & 0xFF
        if tecla == ord("q"):
            break
        elif tecla in (ord("1"), ord("2"), ord("3")):
            nova_missao = int(chr(tecla))
            if nova_missao != missao_atual:
                missao_atual = nova_missao
                resetar_todas_travas()
                print(f"[INFO] Alternado para a Missão {missao_atual}.")
        elif tecla == ord("r"):
            resetar_todas_travas()
            print("[INFO] Trava de alvo resetada manualmente.")
        elif tecla == ord("e"):
            modo_espelho = not modo_espelho
            print(f"[INFO] Modo Espelho alterado para: {modo_espelho}")

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()