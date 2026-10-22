import cv2
import numpy as np

# Inicializa o dicionário e o detector ArUco 5x5 (IDs de 0 a 999)
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_1000)
aruco_params = cv2.aruco.DetectorParameters()
detector_aruco = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

def processar(frame, espelhar=True):
    h, w = frame.shape[:2]
    centro_cam_x, centro_cam_y = w // 2, h // 2

    # 1. Detecta o ArUco na imagem original (para não inverter a matriz binária do código)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = detector_aruco.detectMarkers(gray)
    
    # 2. Espelha a imagem horizontalmente para ficar no modo habitual de webcam
    if espelhar:
        frame = cv2.flip(frame, 1)
    
    if ids is not None:
        for i, m_id in enumerate(ids.flatten()):
            id_num = int(m_id)
            pts = corners[i][0].copy()
            
            # Se a tela foi espelhada, inverte a coordenada X dos cantos do marcador
            if espelhar:
                pts[:, 0] = (w - 1) - pts[:, 0]
                
            # Desenha o contorno verde ao redor do ArUco
            pts_int = pts.astype(np.int32).reshape((-1, 1, 2))
            cv2.polylines(frame, [pts_int], isClosed=True, color=(0, 255, 0), thickness=2)
            
            # Calcula o centro geométrico do marcador na tela
            cx = int(np.mean(pts[:, 0]))
            cy = int(np.mean(pts[:, 1]))
            
            # Calcula o erro de alinhamento em pixels (para o controle do drone)
            erro_x = cx - centro_cam_x
            erro_y = cy - centro_cam_y
            
            # Regra lógica de divisibilidade por 3, 4 ou 5
            divs = [d for d in (3, 4, 5) if id_num % d == 0]
            txt_divs = str(divs) if divs else "Nenhum (3,4,5)"
            
            # Desenha o centro do ArUco e a linha até o centro da câmera
            cv2.circle(frame, (cx, cy), 5, (0, 255, 0), -1)
            cv2.line(frame, (centro_cam_x, centro_cam_y), (cx, cy), (0, 255, 255), 2)
            
            # Exibe o ID, divisibilidade e erro de posição (com letras normais)
            cv2.putText(frame, f"ID: {id_num} | Div por: {txt_divs}", (max(10, cx - 80), max(25, cy - 25)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
            cv2.putText(frame, f"Erro X:{erro_x}px Y:{erro_y}px", (max(10, cx - 80), min(h - 10, cy + 25)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
    else:
        cv2.putText(frame, "Procurando ArUco 5x5...", (10, 65),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        
    return frame

def executar_missao_1():
    cap = cv2.VideoCapture(0)
    print("Missao 1 (ArUco 5x5) iniciada! Pressione 'q' na janela da camera para sair.")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Erro ao acessar a webcam.")
            break

        # Processa a Missão 1 já com o espelhamento ativo (espelhar=True)
        frame = processar(frame, espelhar=True)

        # Desenha a mira central da câmera e o cabeçalho
        h, w = frame.shape[:2]
        cv2.drawMarker(frame, (w // 2, h // 2), (255, 255, 255), cv2.MARKER_CROSS, 20, 1)
        cv2.putText(frame, "MISSAO 1: ArUco 5x5 ('q' para sair)", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 0), 2)

        cv2.imshow("Missao 1 - ArUco 5x5 (Eletroquad SAE)", frame)

        if (cv2.waitKey(1) & 0xFF) == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    executar_missao_1()