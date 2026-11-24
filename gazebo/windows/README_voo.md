# S500: voo pelo teclado no Windows 11

Configuração local validada em 27/10/2026: Gazebo Sim 8.10.0, ArduCopter SITL
oficial para Windows (CopterStable, commit em `binarios_sitl.json`), Python 3.12,
Pymavlink e Pygame. Não precisa de Ubuntu, WSL ou Mission Planner.

## Preparar e iniciar

No PowerShell, na raiz do repositório, com a sessão anterior encerrada:

```powershell
powershell -ExecutionPolicy Bypass -File gazebo/windows/preparar_sitl.ps1
pixi run --manifest-path "%USERPROFILE%\gz-s500\pixi.toml" python gazebo/windows/iniciar_sitl.py --gui --controle
```

O preparo usa o manifesto/lockfile e verifica SHA256 dos binários oficiais. A
pasta padrão é `%USERPROFILE%\gz-s500` (sem espaços). Ajuste o caminho do comando
caso use outro usuário/runtime. A inicialização do estimador/GPS pode levar até
dois minutos. O controle aguarda o estimador, arma sem forçar e decola para 2 m.

Se o servidor já estiver ativo:

```powershell
pixi run --manifest-path "%USERPROFILE%\gz-s500\pixi.toml" python gazebo/windows/iniciar_sitl.py --conectar --gui --controle
```

Clique na janela **S500 - Joystick Virtual** para receber o teclado:

| Tecla | Ação |
|---|---|
| W / S | Frente / trás |
| A / D | Esquerda / direita |
| Seta cima / baixo | Subir / descer |
| Seta esquerda / direita | Girar |
| Q | Zerar comandos de movimento |
| L ou fechar o painel | Pousar |

Ao perder o foco, o painel envia velocidades zero. Ele mostra altitude estimada
pelo autopiloto e distância do TF-Luna; são grandezas diferentes. A leitura do
sensor depende de distância, inclinação e faixa de 0,20–8 m.

Depois de pousar, encerre a sessão:

```powershell
pixi run --manifest-path "%USERPROFILE%\gz-s500\pixi.toml" python gazebo/windows/iniciar_sitl.py --parar
```

Esse comando encerra somente processos registrados da sessão. Não use durante
o voo se quiser preservar um pouso normal: ele interrompe também a física.

## Arquitetura e limites

`gerar_mundo_voo.py` cria uma cópia dinâmica do S500 com origem no centro do
corpo, pés de apoio, IMU/odometria e ordem/sentidos Quad-X dos motores. O modelo
de referência fica preservado. Massa/inércia são simplificadas e o limite de
rotação desta cópia é 1100 rad/s; não constitui calibração do S500 físico.

No Windows, foi implementada uma **ponte Python experimental**, em vez de
compilar o plugin C++ oficial. `ponte_gazebo_json.py` usa a interface JSON oficial:
recebe PWM por UDP 9002, aplica os comandos aos motores no Gazebo e devolve
IMU, posição, orientação, velocidade e a distância do TF-Luna como rangefinder 1
(`rng_1`, com `RNGFND1_TYPE = 100`). GPS, bússola e barômetro do autopiloto
continuam sendo sintetizados pelo próprio SITL a partir da posição enviada (a
interface JSON não tem entradas para eles). A física vem do Gazebo; a
estabilização e os modos de voo vêm do ArduPilot. A ponte transforma FLU/NWU
para FRD/NED, alinhando X do mundo com o norte de simulação, e usa o tempo
simulado.

Partição Gazebo: `s500_voo_local`. MAVLink é enviado para UDP
`127.0.0.1:14550`, usado pelo controle do GitHub. O SITL anuncia também uma
porta TCP secundária (consulte `sitl.log`; neste binário, `5761`). Use um único
leitor na porta UDP por vez. Não há MAVProxy nessa configuração.

Mesa/llvmpipe e Ogre na interface contornam a falha do driver OpenGL AMD
observada nesta máquina. O mundo mantém Ogre2 para o lidar. A renderização por
software pode deixar a simulação abaixo da velocidade real.

## Validação

```powershell
pixi run --manifest-path "%USERPROFILE%\gz-s500\pixi.toml" python python/testes/teste_ponte_gazebo.py
# Com sessão ativa e painel de teclado fechado:
pixi run --manifest-path "%USERPROFILE%\gz-s500\pixi.toml" python python/testes/validar_voo_sitl.py
```

O ensaio integrado aprovado decolou para 2 m, enviou velocidade frontal de
0,4 m/s, parou, pousou e confirmou desarmamento. Avaliou 190 amostras de
telemetria; inclinação máxima nas etapas de avanço/parada ficou em cerca de 5°.
Resumo preservado em `resultado_voo_sitl.json`; logs e amostras completas ficam
em `resultados/logs/sitl/` e `%USERPROFILE%\gz-s500\voo/`.

A primeira tentativa de movimento revelou sentidos de motor invertidos na
cópia de voo e foi rejeitada. Após corrigir a matriz, o ensaio completo passou.
Houve também uma inicialização sem publicação de sensores no Windows; não foi
tratada como teste aprovado. Se não houver heartbeat, confira os três logs.
Os avisos internos de descriptor/log do Gazebo Windows permanecem registrados.

Isso valida uma demonstração local de voo manual. Não valida rádio físico,
hardware, missão de competição ou fidelidade aerodinâmica do airframe.

Fontes: [SITL Windows](https://firmware.ardupilot.org/Tools/MissionPlanner/sitl/CopterStable/),
[protocolo JSON](https://github.com/ArduPilot/ardupilot/blob/master/libraries/SITL/SIM_JSON.cpp),
[matriz de motores](https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Motors/AP_MotorsMatrix.cpp),
[plugin oficial de referência](https://github.com/ArduPilot/ardupilot_gazebo).
