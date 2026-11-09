# Validação ArduCopter SITL

Scripts usados nos ensaios de 01/11/2026 com ArduCopter 4.7.2 e Mission Planner.
O ambiente ensaiado foi o SITL nativo do Windows. O WSL/Ubuntu original não foi
acessado; não se deve atribuir a ele a configuração ou os resultados destes testes.

O [relatório completo](../../docs/validacao_sitl_2026-11-01/RELATORIO_VALIDACAO.txt)
e as [evidências](../../docs/validacao_sitl_2026-11-01/evidencias/) registram os testes
de rádio/GCS, geofence e voo autônomo. Estes utilitários são independentes dos
códigos de missão e controle já presentes no repositório; não comprovam a
validação desses outros programas nem das provas da competição.

## Instalação

No diretório `python/sitl`, usando Python 3.12:

```sh
python -m venv .venv
# Windows PowerShell:
.venv\Scripts\Activate.ps1
# Ubuntu, em uma sessão Ubuntu separadamente configurada:
# source .venv/bin/activate
python -m pip install -r requirements.txt
```

Inicie o SITL pelo Mission Planner, sem marcar **Wipe**, para preservar os
parâmetros. Na instalação utilizada, a conexão do Python é `tcp:127.0.0.1:5762`.
O Mission Planner usa sua conexão separada. Não conecte estes scripts a aeronaves
reais: o escopo de validação é exclusivamente o simulador.

## Consulta e ensaios

Execute um script por vez. A chamada sem `--fly-sitl` apenas consulta os
parâmetros; não arma o veículo nem escreve configurações.

```sh
python voo_autonomo.py --connection tcp:127.0.0.1:5762
python validar_fase3.py --test configure --log configuracao.jsonl
python validar_fase3.py --test rc --log rc.jsonl
python validar_fase3.py --test gcs --log gcs.jsonl
python validar_geofence.py --log geofence.jsonl
python voo_autonomo.py --connection tcp:127.0.0.1:5762 --fly-sitl --log voo.jsonl
python exportar_estado.py --output parametros.json
python -m unittest discover -s tests -v
```

`--test configure` escreve e relê os parâmetros. `--test rc`, `--test gcs`,
`validar_geofence.py` e `--fly-sitl` executam voos simulados. A missão autônoma
comanda 5 m de altitude, deslocamento de 2 m ao norte e pouso LAND. Confirma
altitude a até 0,4 m do alvo, posição a até 0,6 m do alvo e desarmamento final.

No teste GCS, o ID monitorado é temporariamente 254 e seus heartbeats são
interrompidos. O Mission Planner em 255 não mascara essa perda. O ID original é
restaurado após o teste; interromper apenas o Python quando ambos usam 255 não
representa perda de toda a estação de solo.

## Configuração e resultados

O [extrato de parâmetros](configuracao_final.param) contém os nomes/unidades do
firmware 4.7.2: teto de 7 m, margem de 2 m, RTL nominal de 5 m, failsafes RC/GCS
ativos, timeout GCS de 5 s, subida/descida de 0,5 m/s e navegação de 1 m/s.
As checagens de armamento permaneceram ativas e `SIM_RC_FAIL` terminou em zero.

O teste de geofence solicitou decolagem a 10 m. O piloto automático detectou a
violação e executou RTL/pouso. O pico caiu de 9,098 m para 7,331 m com a redução
de velocidade. Portanto, 7 m é o gatilho configurado, não uma barreira física
absoluta. O ensaio não estabelece garantia de ausência de flyaway.

Os scripts tratam nomes antigos/novos, incluindo a inversão de sentido entre
`ARMING_CHECK=1` e `ARMING_SKIPCHK=0`. Não importe os parâmetros cegamente em
outra versão de firmware. Os voos foram concluídos e o estado desarmado foi
confirmado; não foi realizado ensaio de persistência após reiniciar o SITL.

## Retomada no Ubuntu original

A instalação e os ensaios no Ubuntu permanecem pendentes. Depois de localizar e
iniciar o SITL original, acrescente no MAVProxy `output add 127.0.0.1:14551`,
mantendo a saída existente para o Mission Planner. No Ubuntu, crie o ambiente
virtual, instale os requisitos e use `--connection udpin:127.0.0.1:14551` em cada
script. Confira os parâmetros reais antes dos voos. Esses passos são orientação
de continuidade, não evidência de execução no WSL.
