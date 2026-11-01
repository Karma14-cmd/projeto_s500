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
