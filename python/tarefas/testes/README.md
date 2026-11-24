# Testes do S500

| Área | Roteiro ou script | Evidência esperada |
| --- | --- | --- |
| Sensores simulados | [Ensaio dos seis sensores](./testar_sensores_gazebo.py) | Leituras coerentes nas duas poses de teste |
| Infravermelho | [Guia e cenários do TF-Luna](../../../gazebo/README_infravermelho.md) | Distância por cenário e comportamento em perda de leitura |
| Ponte Gazebo / SITL | [Testes da ponte e da câmera](./teste_ponte_gazebo.py) e [validação de voo](./validar_voo_sitl.py) | Resultado dos testes e log da sessão |
| Voo manual em simulação | [Guia Windows](../../../gazebo/windows/README_voo.md) | Decolagem, controle e pouso registrados |
| Segurança e navegação SITL | [Guia dos ensaios](sitl/README.md) e [testes do plano](sitl/tests/test_plan.py) | Resultado de geofence, perda de RC/GCS e plano curto |
| Resultados anteriores | [Relatório de 01/11/2026](sitl/validacao_sitl_2026-11-01/RELATORIO_VALIDACAO.txt) e [evidências](sitl/validacao_sitl_2026-11-01/evidencias/) | Resultados e parâmetros da configuração ensaiada |

Os demais scripts de bancada ficam [nesta pasta](./). Ao registrar um ensaio, anote o commit, o ambiente, o cenário, o resultado esperado, o resultado observado e o caminho do log. Identifique se foi executado em simulação ou em hardware.

## Testes em contêiner Linux

Da raiz do repositório, use o Docker para rodar os testes de cenários, pontuação e plano SITL sem interface gráfica.

```powershell
docker build -f python/tarefas/testes/Dockerfile -t iguanas-testes .
docker run --rm iguanas-testes
```

Para rodar a mesma suíte com Python 3.12 no host:

```powershell
python python/tarefas/testes/test_missoes_2027.py
python python/tarefas/testes/sitl/tests/test_plan.py
```

O ensaio dos seis sensores e o voo integrado usam o ambiente Pixi do [guia do Gazebo](../../../gazebo/windows/README_voo.md). O teste da ponte e da câmera usa os bindings Gazebo desse mesmo ambiente.
