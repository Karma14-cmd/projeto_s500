# Projeto S500

Repositório de modelos e scripts do projeto S500 da equipe Iguanas EletroQuad.

- `gazebo/`: modelos, mundos e documentação do sensor infravermelho.
- `python/comunicacao/`, `python/controle/`, `python/missao/` e `python/legado/`:
  códigos já existentes de comunicação, controle e missões.
- [`python/sitl/`](python/sitl/README.md): utilitários de configuração e validação
  ArduCopter SITL adicionados com os ensaios de 01/11/2026.
- [`docs/validacao_sitl_2026-11-01/`](docs/validacao_sitl_2026-11-01/): relatório,
  parâmetros exportados e registros dos ensaios.

Foram testados geofence, failsafes RC/GCS e uma missão curta de decolagem,
navegação e pouso no SITL nativo do Mission Planner com ArduCopter 4.7.2.
Os resultados não comprovam os códigos de missão anteriores, execução no WSL
original nem operação com aeronave física. Consulte o relatório para critérios,
limitações e passos pendentes.

## Sensores simulados no Gazebo

O modelo [`quadricoptero`](gazebo/modelos/quadricoptero/model.sdf) publica os
sensores abaixo via gz-transport ao ser carregado pelo
[`mundo_s500.sdf`](gazebo/mundos/mundo_s500.sdf):

| Sensor | Tipo SDF | Tópico | Taxa | Posição no corpo (m) |
| --- | --- | --- | --- | --- |
| IMU | `imu` | `/imu` | 200 Hz | centro (0, 0, 0) |
| TF-Luna (infravermelho) | `gpu_lidar` | `/s500/infravermelho/scan` | 100 Hz | (0, 0, -0,10), apontado para baixo |
| GPS | `navsat` | `/s500/gps` | 10 Hz | (0, 0, 0,10) |
| Bússola | `magnetometer` | `/s500/bussola` | 20 Hz | (0, 0, 0,10) |
| Barômetro | `air_pressure` | `/s500/barometro` | 20 Hz | (0, 0, 0,045) |
| Câmera frontal | `camera` | `/s500/camera` | 15 Hz | (0,15, 0, -0,04), 640×480 RGB, FOV 1,15 rad |

O mundo habilita os sistemas `Imu`, `NavSat`, `Magnetometer` e `AirPressure`
e fixa a origem em coordenadas WGS84 (latitude -5,36°, longitude -49,12°,
elevação 100 m, frame ENU). Sem essa origem o GPS e a bússola não geram
leituras georreferenciadas.

Para inspecionar um tópico com a simulação em execução:

```bash
gz topic -l                    # lista os tópicos publicados
gz topic -e -t /s500/gps       # mostra as mensagens do GPS
```
