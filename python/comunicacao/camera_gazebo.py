"""Câmera do S500 no Gazebo com a mesma interface básica de cv2.VideoCapture.

Assina o tópico gz.msgs.Image da câmera frontal (``/s500/camera``) e entrega
quadros BGR em ``numpy.ndarray``, prontos para as funções de visão das missões.
Precisa dos bindings ``gz.transport13``/``gz.msgs10`` e da mesma partição
Gazebo da simulação (variável ``GZ_PARTITION``; a sessão de voo usa
``s500_voo_local``).

Exemplo:
    cap = CameraGazebo()
    ok, frame = cap.read()
"""
import threading
import time

import numpy as np

TOPICO_PADRAO = "/s500/camera"


def imagem_para_bgr(msg):
    """Converte gz.msgs.Image RGB_INT8/BGR_INT8/L_INT8 em array BGR (h, w, 3)."""
    from gz.msgs10 import image_pb2

    largura, altura = msg.width, msg.height
    passo = msg.step or largura * 3
    formato = msg.pixel_format_type
    dados = np.frombuffer(msg.data, dtype=np.uint8)
    if formato == image_pb2.L_INT8:
        cinza = dados.reshape(altura, msg.step or largura)[:, :largura]
        return np.repeat(cinza[:, :, None], 3, axis=2).copy()
    if formato not in (image_pb2.RGB_INT8, image_pb2.BGR_INT8):
        raise ValueError(f"Formato de pixel não suportado: {formato}")
    quadro = dados.reshape(altura, passo)[:, :largura * 3].reshape(altura, largura, 3)
    if formato == image_pb2.RGB_INT8:
        quadro = quadro[:, :, ::-1]
    return np.ascontiguousarray(quadro)


class CameraGazebo:
    """Substituto mínimo de cv2.VideoCapture para a câmera simulada."""

    def __init__(self, topico=TOPICO_PADRAO, espera_s=10.0):
        from gz.transport13 import Node
        from gz.msgs10.image_pb2 import Image

        self.topico = topico
        self.espera_s = espera_s
        self._quadro = None
        self._contador = 0
        self._entregue = 0
        self._cond = threading.Condition()
        self._node = Node()
        self._aberta = self._node.subscribe(Image, topico, self._receber)

    def _receber(self, msg):
        try:
            quadro = imagem_para_bgr(msg)
        except ValueError:
            return
        with self._cond:
            self._quadro = quadro
            self._contador += 1
            self._cond.notify_all()

    def isOpened(self):
        return bool(self._aberta)

    def read(self):
        """Espera um quadro novo (até ``espera_s``); devolve (ok, quadro)."""
        limite = time.monotonic() + self.espera_s
        with self._cond:
            while self._contador == self._entregue:
                restante = limite - time.monotonic()
                if restante <= 0:
                    return False, None
                self._cond.wait(restante)
            self._entregue = self._contador
            return True, self._quadro.copy()

    def set(self, *_):
        """Resolução e taxa são definidas no model.sdf; ignorado."""
        return False

    def release(self):
        if self._aberta:
            self._node.unsubscribe(self.topico)
            self._aberta = False
