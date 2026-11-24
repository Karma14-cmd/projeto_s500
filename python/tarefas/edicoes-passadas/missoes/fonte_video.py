"""Entrada de vídeo compartilhada pelos programas de visão."""
from pathlib import Path
import sys

import cv2


def abrir_fonte(fonte, topico, *, resolucao=None):
    if fonte == "gazebo":
        sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
        from comunicacao.camera_gazebo import CameraGazebo

        return CameraGazebo(topico)

    camera = cv2.VideoCapture(0)
    if resolucao is not None:
        largura, altura = resolucao
        camera.set(cv2.CAP_PROP_FRAME_WIDTH, largura)
        camera.set(cv2.CAP_PROP_FRAME_HEIGHT, altura)
    return camera
