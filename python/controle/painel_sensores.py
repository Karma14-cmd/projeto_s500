"""Painel Pygame para os sensores do S500 no Gazebo e a telemetria do SITL."""
import math
import threading
import time

import pygame


class PainelSensores:
    FUNDO = (14, 22, 34)
    CARTAO = (29, 41, 58)
    TEXTO = (234, 242, 250)
    ROTULO = (132, 210, 231)

    def __init__(self):
        pygame.display.init()
        pygame.font.init()
        self.tela = pygame.display.set_mode((720, 715))
        pygame.display.set_caption("S500 | Sensores ao vivo")
        self.titulo = pygame.font.Font(None, 34)
        self.rotulo = pygame.font.Font(None, 25)
        self.valor = pygame.font.Font(None, 24)

    def texto(self, texto, x, y, fonte, cor):
        self.tela.blit(fonte.render(texto, True, cor), (x, y))

    @staticmethod
    def vetor(dados, casas=2):
        if dados is None:
            return "Aguardando leitura"
        return "  ".join(f"{a.upper()} {dados.get(a, 0):+.{casas}f}" for a in "xyz")

    def atualizar(self, gazebo, sitl, imagem):
        if any(evento.type == pygame.QUIT for evento in pygame.event.get()):
            pygame.display.quit()
            return False

        self.tela.fill(self.FUNDO)
        self.texto("S500  |  Sensores ao vivo", 20, 16, self.titulo, self.TEXTO)

        baro = gazebo.get("barometro", {}).get("pressure")
        gps = gazebo.get("gps")
        imu = gazebo.get("imu", {})
        lidar = gazebo.get("tf_luna")
        distancia = lidar.get("distancia_valida_m") if lidar else None
        altitude = sitl.get("GLOBAL_POSITION_INT", {}).get("relative_alt")
        yaw = sitl.get("ATTITUDE", {}).get("yaw")
        status = sitl.get("HEARTBEAT")
        pronto = sitl.get("EKF_STATUS_REPORT", {}).get("flags", 0) & 19 == 19
        estado = "Aguardando SITL"
        if status:
            estado = "GPS / EKF em inicialização"
        if status and pronto:
            estado = "ARMADO" if (status or {}).get("base_mode", 0) & 128 else "DESARMADO"
        alt = f"{altitude / 1000:.2f} m" if altitude is not None else "—"
        rumo = f"{math.degrees(yaw) % 360:.0f}°" if yaw is not None else "—"
        self.texto(f"SITL   {estado}   Altitude {alt}   Rumo {rumo}",
                   20, 54, self.rotulo, self.ROTULO)

        self.texto("Câmera de bordo", 20, 90, self.rotulo, self.TEXTO)
        tela_camera = pygame.Rect(20, 116, 680, 334)
        pygame.draw.rect(self.tela, self.CARTAO, tela_camera, border_radius=8)
        if imagem is not None and len(imagem.data) == imagem.width * imagem.height * 3:
            quadro = pygame.image.frombuffer(imagem.data, (imagem.width, imagem.height), "RGB")
            escala = min(tela_camera.width / imagem.width, tela_camera.height / imagem.height)
            tamanho = (round(imagem.width * escala), round(imagem.height * escala))
            quadro = pygame.transform.smoothscale(quadro, tamanho)
            self.tela.blit(quadro, quadro.get_rect(center=tela_camera.center))
        else:
            self.texto("Aguardando imagem", 277, 272, self.valor, self.TEXTO)

        aguardando = "Aguardando leitura"
        sem_distancia = "Fora da faixa" if lidar is not None else aguardando
        cartoes = (
            ("IMU · aceleração (m/s²)", self.vetor(imu.get("linear_acceleration"))),
            ("IMU · giroscópio (rad/s)", self.vetor(imu.get("angular_velocity"))),
            ("GPS · latitude / longitude", f"{gps.get('latitude_deg', 0):.7f}°,  {gps.get('longitude_deg', 0):.7f}°" if gps else aguardando),
            ("Bússola · campo (gauss)", self.vetor(gazebo.get("bussola", {}).get("field_tesla"), 3)),
            ("Barômetro · pressão", f"{baro:.2f} Pa" if baro is not None else aguardando),
            ("TF-Luna · distância ao piso", f"{distancia:.2f} m" if distancia is not None else sem_distancia),
        )
        for i, (rotulo, valor) in enumerate(cartoes):
            x = 20 + (i % 2) * 346
            y = 465 + (i // 2) * 79
            pygame.draw.rect(self.tela, self.CARTAO, (x, y, 334, 68), border_radius=8)
            self.texto(rotulo, x + 14, y + 10, self.rotulo, self.ROTULO)
            self.texto(valor, x + 14, y + 38, self.valor, self.TEXTO)
        pygame.display.flip()
        return True


def main():
    from google.protobuf.json_format import MessageToDict
    from gz.msgs10.fluid_pressure_pb2 import FluidPressure
    from gz.msgs10.image_pb2 import Image
    from gz.msgs10.imu_pb2 import IMU
    from gz.msgs10.laserscan_pb2 import LaserScan
    from gz.msgs10.magnetometer_pb2 import Magnetometer
    from gz.msgs10.navsat_pb2 import NavSat
    from gz.transport13 import Node
    from pymavlink import mavutil

    from leitura_infravermelho_s500 import distancia_valida

    painel = PainelSensores()
    relogio = pygame.time.Clock()
    node = Node()
    trava = threading.Lock()
    leituras = {}

    def receber(nome, mensagem):
        with trava:
            leituras[nome] = mensagem

    for nome, topico, tipo in (
        ("imu", "imu", IMU), ("gps", "gps", NavSat),
        ("bussola", "bussola", Magnetometer), ("barometro", "barometro", FluidPressure),
        ("tf_luna", "infravermelho/scan", LaserScan), ("camera", "camera", Image),
    ):
        if not node.subscribe(tipo, f"/s500/{topico}", lambda msg, nome=nome: receber(nome, msg)):
            raise RuntimeError(f"Falha ao assinar /s500/{topico}")

    mav = mavutil.mavlink
    tipos = ("HEARTBEAT", "ATTITUDE", "GLOBAL_POSITION_INT", "EKF_STATUS_REPORT")
    sitl = None
    proxima_tentativa = 0
    pediu_telemetria = False
    try:
        while True:
            agora = time.monotonic()
            if sitl is None and agora >= proxima_tentativa:
                try:
                    sitl = mavutil.mavlink_connection("tcp:127.0.0.1:5761", source_system=254, retries=0)
                except OSError:
                    proxima_tentativa = agora + 2

            dados_sitl = {}
            if sitl is not None:
                try:
                    limite = time.monotonic() + .02
                    while time.monotonic() < limite and sitl.recv_msg() is not None:
                        pass
                    if sitl.target_system and not pediu_telemetria:
                        sitl.mav.request_data_stream_send(sitl.target_system, sitl.target_component,
                                                          mav.MAV_DATA_STREAM_ALL, 5, 1)
                        pediu_telemetria = True
                    dados_sitl = {tipo: sitl.messages[tipo].to_dict() for tipo in tipos if tipo in sitl.messages}
                except OSError:
                    sitl.close()
                    sitl = None
                    pediu_telemetria = False
                    proxima_tentativa = agora + 2

            with trava:
                quadro = leituras.copy()
            imagem = quadro.pop("camera", None)
            gazebo = {nome: MessageToDict(msg, preserving_proto_field_name=True) for nome, msg in quadro.items()}
            if "tf_luna" in quadro:
                gazebo["tf_luna"]["distancia_valida_m"] = distancia_valida(quadro["tf_luna"])
            if not painel.atualizar(gazebo, dados_sitl, imagem):
                break
            relogio.tick(10)
    finally:
        if sitl is not None:
            sitl.close()
        pygame.quit()


if __name__ == "__main__":
    main()
