import cv2
import numpy as np

# Variáveis de estado para o Modo de Certeza e Rastreamento
_certeza = 0
_travado = False
_pos_travada = None  # Guarda (cx, cy, raio)
_frames_sem_ver = 0

# Histórico vetorial (seno e cosseno) para estabilizar a agulha sem saltos
_hist_sen = []
_hist_cos = []
_ultimo_ponto_agulha = None
_ultimo_angulo = None

def resetar_trava():
    """Destrava o manómetro manualmente (Tecla 'r')."""
    global _certeza, _travado, _pos_travada, _frames_sem_ver
    global _hist_sen, _hist_cos, _ultimo_ponto_agulha, _ultimo_angulo
    _certeza = 0
    _travado = False
    _pos_travada = None
    _frames_sem_ver = 0
    _hist_sen.clear()
    _hist_cos.clear()
    _ultimo_ponto_agulha = None
    _ultimo_angulo = None

def processar(frame, espelhar=True):
    global _certeza, _travado, _pos_travada, _frames_sem_ver
    global _hist_sen, _hist_cos, _ultimo_ponto_agulha, _ultimo_angulo

    # 1. Espelha a imagem para o modo habitual da webcam
    if espelhar:
        frame = cv2.flip(frame, 1)

    h, w = frame.shape[:2]
    centro_cam_x, centro_cam_y = w // 2, h // 2

    # 2. Pré-processamento por alto contraste (fundo branco do mostrador vs borda preta)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    _, thresh_mostrador = cv2.threshold(blur, 130, 255, cv2.THRESH_BINARY)

    contours, _ = cv2.findContours(thresh_mostrador, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # 3. Lógica de Busca e Perseguição do Mostrador
    if not _travado:
        melhor_alvo = None
        maior_area = 0

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 2000:
                perimetro = cv2.arcLength(cnt, True)
                if perimetro > 0:
                    circularidade = 4 * np.pi * (area / (perimetro ** 2))
                    if circularidade > 0.65 and area > maior_area:
                        (x, y), raio = cv2.minEnclosingCircle(cnt)
                        if raio > 35:
                            maior_area = area
                            melhor_alvo = (int(x), int(y), int(raio))

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
        # MODO TRAVADO: Segue o mostrador mesmo com o movimento da mão
        lx, ly, lr = _pos_travada
        melhor_candidato = None
        menor_custo = float('inf')

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 1200:
                perimetro = cv2.arcLength(cnt, True)
                if perimetro > 0:
                    circularidade = 4 * np.pi * (area / (perimetro ** 2))
                    # Tolerância maior para suportar inclinação e movimento do telemóvel
                    if circularidade > 0.45:
                        (nx, ny), n_raio = cv2.minEnclosingCircle(cnt)
                        if n_raio > 30:
                            dist_centro = np.hypot(nx - lx, ny - ly)
                            custo = dist_centro - (area * 0.005)
                            if custo < menor_custo:
                                menor_custo = custo
                                melhor_candidato = (nx, ny, n_raio)

        if melhor_candidato is not None:
            nx, ny, n_raio = melhor_candidato
            novo_x = int(0.25 * lx + 0.75 * nx)
            novo_y = int(0.25 * ly + 0.75 * ny)
            novo_r = int(0.30 * lr + 0.70 * n_raio)
            _pos_travada = (novo_x, novo_y, novo_r)
            _frames_sem_ver = 0
        else:
            _frames_sem_ver += 1
            if _frames_sem_ver > 60:
                resetar_trava()

    # 4. Leitura da Agulha e Desenho na Tela
    if _pos_travada is not None:
        cx, cy, r = _pos_travada
        desvio_x = cx - centro_cam_x
        desvio_y = cy - centro_cam_y

        # Isola a parte interna do mostrador (78% do raio para ignorar a borda preta)
        mask_interna = np.zeros_like(gray)
        cv2.circle(mask_interna, (cx, cy), int(r * 0.78), 255, -1)

        # Calcula o brilho médio do mostrador para adaptar o limiar da agulha automaticamente
        brilho_medio = cv2.mean(gray, mask=mask_interna)[0]
        limiar_agulha = max(50, min(160, int(brilho_medio * 0.68)))

        _, thresh_interno = cv2.threshold(gray, limiar_agulha, 255, cv2.THRESH_BINARY_INV)
        needle_img = cv2.bitwise_and(thresh_interno, thresh_interno, mask=mask_interna)

        # Procura retas que representem o ponteiro
        lines = cv2.HoughLinesP(
            needle_img, rho=1, theta=np.pi / 180, threshold=18,
            minLineLength=int(r * 0.28), maxLineGap=12
        )

        melhor_ponta = None
        maior_alcance = 0

        if lines is not None:
            for line in lines.reshape(-1, 4):
                x1, y1, x2, y2 = int(line[0]), int(line[1]), int(line[2]), int(line[3])
                d1 = float(np.hypot(x1 - cx, y1 - cy))
                d2 = float(np.hypot(x2 - cx, y2 - cy))

                # A agulha precisa ter uma ponta perto do centro (< 35% do raio) e outra longe
                dist_base = min(d1, d2)
                dist_ponta = max(d1, d2)

                if dist_base < (r * 0.35) and dist_ponta > maior_alcance:
                    maior_alcance = dist_ponta
                    melhor_ponta = (x1, y1) if d1 > d2 else (x2, y2)

        if melhor_ponta is not None:
            px, py = melhor_ponta
            ang_rad = float(np.arctan2(py - cy, px - cx))

            # Média vetorial (evita pulos quando o ângulo passa de 359° para 0°)
            _hist_sen.append(np.sin(ang_rad))
            _hist_cos.append(np.cos(ang_rad))
            if len(_hist_sen) > 6:
                _hist_sen.pop(0)
                _hist_cos.pop(0)

            ang_suave_rad = float(np.arctan2(np.mean(_hist_sen), np.mean(_hist_cos)))
            _ultimo_angulo = ( float(np.degrees(ang_suave_rad)) + 360.0 ) % 360.0

            # Calcula a ponta suavizada da agulha para o desenho não tremer
            comp_agulha = int(r * 0.72)
            ponta_x = int(cx + comp_agulha * np.cos(ang_suave_rad))
            ponta_y = int(cy + comp_agulha * np.sin(ang_suave_rad))
            _ultimo_ponto_agulha = (ponta_x, ponta_y)

        # Desenho do mostrador e status de trava
        cor_mostrador = (0, 255, 0) if _travado else (255, 0, 0)
        if _travado:
            status_txt = "MANOMETRO TRAVADO (100%)" if _frames_sem_ver == 0 else "MEMORIA MANOMETRO (100%)"
            cv2.rectangle(frame, (cx - r - 8, cy - r - 8), (cx + r + 8, cy + r + 8), (0, 255, 0), 2)
        else:
            status_txt = f"Confirmando... {_certeza}%"

        cv2.circle(frame, (cx, cy), r, cor_mostrador, 2)
        cv2.circle(frame, (cx, cy), 5, (0, 255, 0), -1)
        cv2.line(frame, (centro_cam_x, centro_cam_y), (cx, cy), (0, 255, 255), 2)

        # Desenha a agulha estabilizada e o ângulo
        if _ultimo_ponto_agulha is not None and _ultimo_angulo is not None:
            cv2.line(frame, (cx, cy), _ultimo_ponto_agulha, (0, 0, 255), 3)
            txt_ang = f"Angulo: {_ultimo_angulo:.1f} deg"
        else:
            txt_ang = "Buscando agulha..."

        cv2.putText(frame, f"{status_txt} | {txt_ang}", (max(10, cx - 130), max(25, cy - r - 15)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)
        cv2.putText(frame, f"Desvio X: {desvio_x}px Y: {desvio_y}px", (max(10, cx - 80), min(h - 10, cy + r + 25)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
    else:
        cv2.putText(frame, "Aguardando Manometro (Contraste)...", (10, 65),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

    return frame

def executar_missao_3():
    cap = cv2.VideoCapture(0)
    print("Missao 3 iniciada! Pressione 'r' para resetar a trava ou 'q' para sair.")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Erro ao acessar a webcam.")
            break

        frame = processar(frame, espelhar=True)

        h, w = frame.shape[:2]
        cv2.drawMarker(frame, (w // 2, h // 2), (255, 255, 255), cv2.MARKER_CROSS, 20, 1)
        cv2.putText(frame, "MISSAO 3: Manometro ('r' resetar | 'q' sair)", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

        cv2.imshow("Missao 3 - Manometro (Eletroquad SAE)", frame)

        tecla = cv2.waitKey(1) & 0xFF
        if tecla == ord('q'):
            break
        elif tecla == ord('r'):
            resetar_trava()

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    executar_missao_3()