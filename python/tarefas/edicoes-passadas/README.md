# Edições passadas

| Edição | Desafios de referência | Regulamento |
| --- | --- | --- |
| 2025 | Slalom e Hang the Hook | [SAE BRASIL, rev. 4.0](https://arquivos.saebrasil.org.br/2025/EletroQuad/RegulamentoEletroQuadSAEBRASIL-Eletrobras2025_Rev4.0.pdf) |
| 2026 | Bouncing 2.0, Hang the Right Wire e Faulty or Not? | [SAE BRASIL, regras e revisões](https://legado.saebrasil.org.br/programas-estudantis/eletroquad/regras-e-relatorios/) |

## Protótipos existentes

O repositório mantém protótipos de [ArUco](missoes/missao1_aruco.py), [esfera](missoes/missao2_esfera.py) e [manômetro](missoes/missao3_manometro.py), além do [programa integrador](missoes/main_principal.py). A numeração nos nomes dos scripts é histórica. O vínculo entre cada protótipo e uma edição específica ainda precisa de confirmação.

Os scripts de visão aceitam webcam ou a câmera do Gazebo. Consulte o [planejamento de 2027](../2027/README.md) para acompanhar a adaptação e a validação necessárias para as missões atuais.

## Rodar os programas de visão

Os scripts compartilham o leitor de vídeo. O programa integrado usa os detectores de esfera e manômetro dos scripts individuais e mantém a própria versão do rastreamento ArUco.

Para usar webcam, crie um ambiente virtual Python 3.12 e instale [os requisitos de visão](missoes/requirements.txt). Para usar a câmera simulada no Windows, ative primeiro o ambiente Pixi do [guia do Gazebo](../../../gazebo/windows/README_voo.md) e crie o ambiente de visão com acesso aos bindings instalados no Pixi. Em PowerShell, a partir da raiz do repositório, rode `pixi shell --manifest-path "$env:USERPROFILE/gz-s500/pixi.toml"` se for usar Gazebo. Em seguida, instale os requisitos:

```powershell
python -m venv --system-site-packages .venv-visao
.venv-visao/Scripts/python.exe -m pip install -r python/tarefas/edicoes-passadas/missoes/requirements.txt
.venv-visao/Scripts/python.exe python/tarefas/edicoes-passadas/missoes/main_principal.py
```

Com a simulação de voo ativa, defina `$env:GZ_PARTITION = 's500_voo_local'` e acrescente `--fonte gazebo` ao último comando. Para webcam, use o comando como está.
