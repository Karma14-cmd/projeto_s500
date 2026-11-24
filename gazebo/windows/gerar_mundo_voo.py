"""Gera uma cópia dinâmica do S500 para SITL; preserva o modelo de referência."""
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

RAIZ = Path(__file__).resolve().parents[2]


def gerar(destino):
    sdf = ET.parse(RAIZ / 'gazebo/mundos/mundo_s500.sdf')
    world = sdf.getroot().find('world')
    world.remove(world.find('include'))
    model = ET.parse(RAIZ / 'gazebo/modelos/quadricoptero/model.sdf').getroot().find('model')
    ET.SubElement(model, 'pose').text = '0 0 0.31 0 0 0'
    # Origem do corpo no centro de massa, e não 0.5 m acima do modelo.
    for link in model.findall('link'):
        pose = [float(v) for v in link.findtext('pose').split()]
        pose[2] -= .5
        link.find('pose').text = ' '.join(str(v) for v in pose)
    for uri in model.findall('.//mesh/uri'):
        uri.text = (RAIZ / 'gazebo/modelos' / uri.text.removeprefix('model://')).as_posix()
    imu = model.find("link[@name='base_link']/sensor[@type='imu']")
    imu.find('topic').text = '/s500/imu'
    imu.find('update_rate').text = '400'
    # ArduCopter Quad-X: canais 1 FR/ccw, 2 RL/ccw, 3 FL/cw, 4 RR/cw.
    canais = [2, 1, 3, 0]
    for i, motor in enumerate(model.findall('plugin')):
        motor.find('actuator_number').text = str(canais[i])
        motor.find('turningDirection').text = ['cw', 'ccw', 'cw', 'ccw'][i]
        motor.find('maxRotVelocity').text = '1100'
    # Trem de pouso simples, para manter o sensor acima do piso.
    base = model.find("link[@name='base_link']")
    for x, y in [(.14,.14),(-.14,.14),(-.14,-.14),(.14,-.14)]:
        for tag in ['collision', 'visual']:
            item = ET.SubElement(base, tag, name=f'pe_{x}_{y}')
            ET.SubElement(item, 'pose').text = f'{x} {y} -0.175 0 0 0'
            box = ET.SubElement(ET.SubElement(item, 'geometry'), 'box')
            ET.SubElement(box, 'size').text = '.025 .025 .27'
    odom = ET.SubElement(model, 'plugin', filename='gz-sim-odometry-publisher-system',
                         name='gz::sim::systems::OdometryPublisher')
    for key, value in {'odom_publish_frequency':'400', 'dimensions':'3',
                       'odom_topic':'/model/quadricoptero/odometry'}.items():
        ET.SubElement(odom, key).text = value
    world.append(model)
    sdf.write(destino, encoding='utf-8', xml_declaration=True)


if __name__ == '__main__':
    gerar(sys.argv[1])
