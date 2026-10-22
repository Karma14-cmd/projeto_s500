import cv2
import numpy as np

# Diâmetro real da esfera segundo o regulamento (25 cm = 0.25 m)
DIAMETRO_REAL_M = 0.25

# Variáveis de estado para o Rastreamento com Certeza (Target Lock)
_certeza = 0
_travado = False
_pos_travada = None  # Guarda (cx, cy, raio, circularidade)
_frames_sem_ver = 0

def resetar_trava():
    """Destrava o alvo manualmente (Tecla 'r')."""
    global _certeza, _travado, _pos_travada, _frames_sem_ver
    _certeza = 0
    _travado = False
    _pos_travada = None
    _frames_sem_ver = 0

def processar(frame, espelhar=True, focal_px=600.0):
    global _certeza, _travado, _pos_travada, _frames_sem_ver

    # 1. Espelha a imagem para o modo habitual da webcam
    if espelhar:
        frame = cv2.flip(frame, 1)

    h, w = frame.shape[:2]
    centro_cam_x, centro_cam_y = w // 2, h // 2

    # 2. Filtro HSV ampliado para suportar o brilho da tela do celular em movimento
    blurred = cv2.GaussianBlur(frame, (7, 7), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    lower_orange = np.array([2, 70, 70])
    upper_orange = np.array([28, 255, 255])
    mask = cv2.inRange(hsv, lower_orange, upper_orange)

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # 3. Lógica de Busca e Perseguição do Alvo
    if not _travado:
        # FASE 1: Buscando e acumulando certeza
        melhor_alvo = None
        maior_area = 0

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 200:
                perimetro = cv2.arcLength(cnt, True)
                if perimetro > 0:
                    circularidade = 4 * np.pi * (area / (perimetro ** 2))
                    if circularidade > 0.60 and area > maior_area:
                        maior_area = area
                        (x, y), raio = cv2.minEnclosingCircle(cnt)
                        melhor_alvo = (int(x), int(y), int(raio), circularidade)

        if melhor_alvo is not None:
            _pos_travada = melhor_alvo
            _certeza = min(100, _certeza + 10)
            if _certeza >= 100:
                _travado = True
                _frames_sem_ver = 0
        else:
            _certeza = max(0, _certeza - 5)
            if _certeza == 0:
                _pos_travada = None

    else:
        # FASE 2: ALVO TRAVADO — Acompanha a esfera em qualquer movimento na tela
        lx, ly, lr, lc = _pos_travada
        melhor_candidato = None
        menor_custo = float('inf')

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 150:
                perimetro = cv2.arcLength(cnt, True)
                if perimetro > 0:
                    circularidade = 4 * np.pi * (area / (perimetro ** 2))
                    # Tolerância maior (0.42) para não perder o alvo com o borrão de movimento da mão
                    if circularidade > 0.42:
                        (nx, ny), n_raio = cv2.minEnclosingCircle(cnt)
                        dist_centro = np.hypot(nx - lx, ny - ly)

                        # Prioriza o contorno laranja mais próximo da última posição conhecida
                        custo = dist_centro - (area * 0.01)
                        if custo < menor_custo:
                            menor_custo = custo
                            melhor_candidato = (nx, ny, n_raio, circularidade)

        if melhor_candidato is not None:
            nx, ny, n_raio, n_circ = melhor_candidato
            # Resposta rápida (75% posição nova, 25% anterior) para acompanhar o celular sem atraso
            novo_x = int(0.25 * lx + 0.75 * nx)
            novo_y = int(0.25 * ly + 0.75 * ny)
            novo_r = int(0.30 * lr + 0.70 * n_raio)
            _pos_travada = (novo_x, novo_y, novo_r, n_circ)
            _frames_sem_ver = 0
        else:
            # Se o reflexo do celular esconder a cor por um instante, mantém a trava na última posição
            _frames_sem_ver += 1
            # Só destrava sozinho se tirar o celular da frente da câmera por ~2 segundos (60 frames)
            if _frames_sem_ver > 60:
                resetar_trava()

    # 4. Desenho do HUD e informações na tela
    if _pos_travada is not None:
        cx, cy, raio, circularidade = _pos_travada
        desvio_x = cx - centro_cam_x
        desvio_y = cy - centro_cam_y
        diametro_px = max(1, 2 * raio)
        distancia_m = (DIAMETRO_REAL_M * focal_px) / diametro_px

        cor_alvo = (0, 255, 0) if _travado else (0, 165, 255)
        if _travado:
            status_txt = "RASTREANDO ALVO (100%)" if _frames_sem_ver == 0 else "MEMORIA DE ALVO (100%)"
            cv2.rectangle(frame, (cx - raio - 8, cy - raio - 8),
                          (cx + raio + 8, cy + raio + 8), (0, 255, 0), 2)
        else:
            status_txt = f"Confirmando... {_certeza}%"

        cv2.circle(frame, (cx, cy), raio, cor_alvo, 3)
        cv2.circle(frame, (cx, cy), 5, (0, 255, 0), -1)
        cv2.line(frame, (centro_cam_x, centro_cam_y), (cx, cy), (0, 255, 255), 2)

        texto_topo = f"{status_txt} | Dist: {distancia_m:.2f}m"
        texto_base = f"Desvio X: {desvio_x}px Y: {desvio_y}px"

        cv2.putText(frame, texto_topo, (max(10, cx - 120), max(25, cy - raio - 15)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, cor_alvo, 2)
        cv2.putText(frame, texto_base, (max(10, cx - 80), min(h - 10, cy + raio + 25)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
    else:
        cv2.putText(frame, "Aguardando Esfera Laranja...", (10, 65),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)

    return frame

def executar_missao_2():
    cap = cv2.VideoCapture(0)
    print("Missao 2 iniciada! Pressione 'r' para resetar a trava ou 'q' para sair.")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Erro ao acessar a webcam.")
            break

        frame = processar(frame, espelhar=True)

        h, w = frame.shape[:2]
        cv2.drawMarker(frame, (w // 2, h // 2), (255, 255, 255), cv2.MARKER_CROSS, 20, 1)
        cv2.putText(frame, "MISSAO 2: Esfera ('r' resetar | 'q' sair)", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

        cv2.imshow("Missao 2 - Esfera Laranja (Eletroquad SAE)", frame)

        tecla = cv2.waitKey(1) & 0xFF
        if tecla == ord('q'):
            break
        elif tecla == ord('r'):
            resetar_trava()

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    executar_missao_2()