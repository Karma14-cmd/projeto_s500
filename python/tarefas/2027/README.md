# EletroQuad 2027

Planejamento baseado no Regulamento EletroQuad 2027, versão 1.0, de 26/10/2026. A equipe deve conferir a revisão vigente antes de usar os critérios em competição. Os itens abaixo são tarefas de implementação e validação.

| Missão | Referência | Checklist |
| --- | --- | --- |
| M1 — Alta Tensão, A Prova do Gancho | § 5.6.4.2, Figura 2 | [M1](m1-prova-do-gancho.md) |
| M2 — Decisão Estratégica de Rota | § 5.6.4.3, Figura 3 e Tabela 2 | [M2](m2-decisao-estrategica-de-rota.md) |

## Preparação comum

- [ ] Conferir os modos permitidos, as regras de inspeção e a validade das tentativas no § 5.6.
- [ ] Validar a estação de solo com QGroundControl e revisar a montagem e os sensores segundo §§ 5.1.2 e 5.3.
- [ ] Ensaiar o procedimento de inspeção e o setup, registrando o tempo de cada etapa.
- [ ] Conferir o limite de altura, o acesso à USB da controladora e o procedimento para perda de telemetria.
- [ ] Registrar tempo de voo, telemetria, vídeo e resultado por tentativa. Guardar o vínculo com o [roteiro de testes](../testes/README.md).

## Cenários de treino

A partir da raiz do repositório, gere um SDF e inicie-o com o [SITL no Windows](../../../gazebo/windows/README_voo.md). O gerador usa os modelos versionados e grava cenários em `resultados/`.

```powershell
python python/tarefas/2027/gerar_cenario.py m1 --alvo direita --saida resultados/m1.sdf
python python/tarefas/2027/gerar_cenario.py m2 --barreiras 0 --saida resultados/mundo_m2.sdf
pixi run --manifest-path "$env:USERPROFILE/gz-s500/pixi.toml" python gazebo/windows/iniciar_sitl.py --mundo resultados/m1.sdf --gui --controle
```

Encerre a sessão com `--parar` antes de trocar de cenário. O controle gráfico do guia inicia voo a 2 m. A M2 continua exigindo que o operador monitore o teto de 3 m durante o ensaio.

- **M1** inclui duas mangueiras a 1,7 m, esfera selecionável com `--alvo`, três bases de pouso e um gancho dinâmico acoplado ao S500 com uma junta destacável. Use o tópico de soltura somente na sessão de simulação. As posições e os comprimentos das mangueiras são parâmetros de treino.
- **M2** inclui as zonas coloridas da Figura 3 com letras e fronteiras brancas no piso e uma base azul com eixos X e Y no ponto (1, 9). Sem barreiras por padrão, aceita até quatro painéis em posições de treino. Se houver painéis, o primeiro exibe um gabarito configurável com `--gabarito A=40,B=-20,C=40,D=20,E=-20`. O cálculo de rota recebe uma transcrição manual. A leitura por imagem e o desvio de barreiras seguem em aberto.

### Soltar o gancho no Gazebo

Em um terminal ativado com o mesmo Pixi, selecione a partição da sessão e publique uma mensagem vazia na hora da soltura. O tópico de estado informa a condição da junta. Consulte a [documentação da junta destacável](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1DetachableJoint.html).

```powershell
$env:GZ_PARTITION = 's500_voo_local'
gz topic -t /s500/gancho/soltar -m gz.msgs.Empty -p ''
```

### Voo simulado da M2 sem barreiras

O terminal mostra as oito rotas, numeradas de 1 a 8. Escolha uma delas para decolar ou digite 0 para cancelar. O script decola a 2 m, confere as zonas pela odometria do Gazebo, pousa em G e confirma o desarmamento. Antes do teste, encerre outras sessões do S500 que estejam usando as portas UDP 9002 ou 14550.

```powershell
$manifesto = "$env:USERPROFILE/gz-s500/pixi.toml"
pixi run --manifest-path $manifesto --frozen python -B -u python/tarefas/2027/testar_m2_gazebo.py
```

O comando abre o Gazebo com vista de cima e o painel Pygame, que mostra a câmera, os sensores e a telemetria ao vivo. A grade tem quadrados de 1 m e marcas numeradas a cada 3 m. O painel lê a porta TCP 5761, enquanto a rota usa UDP 14550.

Cada tentativa tem uma pasta em `resultados/m2-voos/` com o SDF, as malhas do piso, os logs dos processos e `m2_rN_voo.jsonl` quando o voo chega à conexão MAVLink. O evento `concluido` registra a rota escolhida, o percurso observado, a maior altura do centro do drone acima do piso, a posição de pouso e a duração em tempo real.

Use `--so-preparar` para verificar o SDF e listar as rotas sem iniciar o simulador. Use `--sem-janela` para ensaiar sem interface gráfica e `--rota R2` para escolher antecipadamente uma rota.

### Calcular e registrar a rota M2

```powershell
python python/tarefas/2027/m2_rotas.py --gabarito A=40,B=-20,C=40,D=20,E=-20 --saida resultados/m2.txt
python python/tarefas/2027/m2_rotas.py --gabarito A=40,B=-20,C=40,D=20,E=-20 --rota R2 --percurso SACD --segundos 52 --altura-maxima 2.8
```

Sem `--gabarito`, o cálculo usa o Nível 1. Com `--percurso`, informe a sequência observada até o pouso para aplicar a completude e os limites do § 5.6.4.3. O cálculo usa os custos exibidos na tabela do regulamento, arredondados a uma casa decimal.
