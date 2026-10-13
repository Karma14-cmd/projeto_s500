# Leitura do infravermelho para a 3ª prova

O documento da equipe [Estruturação Elétrica e Automação do Drone S500](https://drive.google.com/file/d/1a5ML_EFIIF-oGjTneS32-iGpOjMUlLH_/view)
identifica o TF-Luna para medição de altura. Como não foi localizado um modelo
SDF do sensor no material consultado, foi usado o **gpu_lidar nativo do Gazebo**
como aproximação geométrica, com parâmetros do fabricante.

## Parâmetros no Gazebo

Edite o sensor `tf_luna` dentro de `gazebo/modelos/quadricoptero/model.sdf`.

| Parâmetro | Valor | Campo SDF |
| --- | --- | --- |
| Alcance mínimo | 0,20 m | `lidar/range/min` |
| Alcance máximo nominal | 8,0 m | `lidar/range/max` |
| Resolução linear | 0,01 m | `lidar/range/resolution` |
| Frequência de aquisição | 100 Hz | `update_rate` |
| FoV aproximado | 2° horizontal e vertical | `scan/*/min_angle` e `max_angle`, em radianos |
| Amostragem do FoV | 3 × 3 raios | `scan/*/samples` |
| Montagem escolhida | 0,10 m abaixo de `base_link`, olhando para baixo | `pose` |
| Ruído | Desabilitado | `lidar/noise/type` |
| Tópico | `/s500/infravermelho/scan` | `topic` |

A grade de raios aproxima o FoV óptico com uma área quadrada. O programa retorna
a menor distância da grade, em metros, arredondada para centímetros. Trata-se de
uma aproximação para desenvolver a leitura; reflexividade, luz ambiente e os erros
do equipamento real não estão simulados. O alcance de 8 m do manual corresponde a
alvo branco com 90% de refletividade em ambiente interno; para alvos escuros, o
alcance é menor. A distância é medida desde o sensor ao longo dos raios e não é
uma estimativa de altitude corrigida pela inclinação da aeronave.

## Execução prevista no Gazebo Harmonic (Linux)

Com Gazebo Harmonic instalado, os bindings Python utilizados são
`python3-gz-transport13` e `python3-gz-msgs10`. Use o Python desse ambiente.
Os comandos abaixo devem ser executados na raiz do repositório.

Terminal 1:

```bash
export GZ_SIM_RESOURCE_PATH="$PWD/gazebo/modelos${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
gz sim -r gazebo/mundos/mundo_s500.sdf
```

Terminal 2:

```bash
python3 python/controle/leitura_infravermelho_s500.py
```

Para inspecionar diretamente a publicação:

```bash
gz topic -e -t /s500/infravermelho/scan
```

O Python lê `gz.msgs.LaserScan` diretamente pelo Gazebo Transport. A frequência de
exibição padrão é 10 Hz; a aquisição no SDF é 100 Hz. O timeout detecta ausência de
mensagens, inclusive quando a simulação está pausada. Medições inválidas, NaN e
infinidades não são apresentadas como distâncias válidas.

O mundo existente contém um drone dinâmico: sem controle de voo, ele cai e o chão
pode entrar na zona cega de 20 cm. Para o ensaio inicial, conferir as primeiras
leituras antes da queda ou manter a aeronave suspensa no cenário de teste. Na pose
inicial e nivelada, a distância geométrica esperada ao chão é aproximadamente
1,40 m (1,0 m do include + 0,5 m do base_link - 0,1 m da montagem).

## Validação pendente

Ainda será testado no Gazebo. Conferir a publicação do tópico, a orientação dos
raios, a leitura em alturas conhecidas e as situações abaixo de 0,20 m e acima de
8 m. O `Sensors` com Ogre2 já está configurado no mundo. O fechamento do bloco
`physics` foi corrigido para deixar chão e luz como filhos de `world`.

## Referências externas

- [Manual TF-Luna da Benewake, revisão 01/04/2024](https://en.benewake.com/uploadfiles/2025/04/20250430174515390.pdf): alcance, resolução, FoV e frequência padrão.
- [Sensores no Gazebo Harmonic](https://gazebosim.org/docs/harmonic/sensors/): uso de `gpu_lidar`, plugin `Sensors` e mensagem `LaserScan`.
- [Gazebo Transport 13 em Python](https://gazebosim.org/api/transport/13/python.html): assinatura de tópicos com os bindings Python.
