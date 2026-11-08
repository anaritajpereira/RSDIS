# RSDIS — Busca e salvamento marítimo multi-robô

Um drone (`drone`) procura o barco à deriva (`barco`, com um painel vermelho) e um robô marítimo (`robo_maritimo`) vai ao encontro dele. A simulação corre em Gazebo.

**Nomes.** No nosso código o WAM-V que controlamos chama-se `robo_maritimo` e o navio à deriva chama-se `barco`. No Gazebo continuam a ser `barco_1` e `barco_2`, porque o mundo vem da pasta `oceano_corrente`, que não alteramos. Esses nomes só aparecem como parâmetros: `entidades` no `poses`, e `modelo_gazebo`, `topico_cmd_vel` e `modelo_alvo` no `robo_maritimo_missao`. O `barco_teleop.py` da simulação só ouve `/barco_1/cmd_vel`, por isso esse tópico mantém o nome.

> Estado: **fase 4**. O gestor arranca a missão. O drone procura o alvo em faixas, deteta-o pela cor e acompanha-o; se o perder, procura-o numa espiral à volta de onde ele deve estar. Um estimador ajusta a posição e a deriva do alvo. O robô robô recebe a ação `Resgatar`, interceta o alvo (ou persegue-o), confirma que chegou e fica a acompanhá-lo. Nessa altura o drone regressa (hand-off). Na **fase 4** o robô engata o alvo e reboca-o até à zona de entrega (ação `Rebocar`). Tudo fica gravado em CSV.

## Requisitos

Ubuntu 20.04, ROS 2 Galactic, Ignition Gazebo Fortress, o simulador MBZIRC (`mbzirc_ign` e `ros_ign_bridge`) e `python3-opencv`. Tudo isto vem na imagem descrita em `ambiente/Containerfile`. A imagem parte da `osrf/mbzirc:mbzirc_sim_latest` e desliga os repositórios apt de 2022, que já não funcionam.

```bash
podman build -t mbzirc_corrigido -f ambiente/Containerfile ambiente
distrobox create --name mbzirc --image localhost/mbzirc_corrigido:latest --nvidia   # sem NVIDIA: tirar --nvidia
```

Dentro da box, carregar o ambiente (por exemplo no `~/.bashrc`, num bloco `if [[ "$CONTAINER_ID" == "mbzirc" ]]`):

```bash
source /opt/ros/galactic/setup.bash
source /home/developer/mbzirc_ws/install/setup.bash
export ROS_DOMAIN_ID=42   # isola dos ROS instalados no host
```

## Estrutura

```
oceano_corrente/oceano_corrente/   ambiente de simulação (não alterar)
src/missao_interfaces/             mensagens DetecaoAlvo, EstimativaAlvo e ações Resgatar e Rebocar
src/missao/                        nós da missão (Python), missao.launch.py e config/missao.yaml
```

## Compilar e correr

Dentro da distrobox, com o MBZIRC carregado (`source`) e **a partir da raiz do repositório**:

```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch missao missao.launch.py                  # com a janela do Gazebo
ros2 launch missao missao.launch.py headless:=true   # só o servidor
```

Argumentos do launch:

| Argumento | Omissão | Descrição |
|---|---|---|
| `oceano_launch` | `./oceano_corrente/oceano_corrente/launch/oceano.launch.py` | caminho do launch da simulação, relativo à pasta onde se corre o comando |
| `headless` | `false` | sem janela do Gazebo |
| `corrente` | `true` | arrancar o nó da corrente |
| `busca_automatica` | `true` | `false` = a missão só arranca com `ros2 service call /missao/iniciar std_srvs/srv/Trigger` |
| `modo_intercecao` | `intercetar` | `intercetar` (vai ao ponto onde o alvo vai estar) ou `perseguir` (vai sempre à posição atual) |
| `reboque` | `true` | `false` = sem fase 4: o robô fica a acompanhar o alvo |
| `pasta_metricas` | `./metricas` | onde o `registo_metricas` grava os CSV |
| `parametros` | `config/missao.yaml` do pacote | ficheiro YAML com os parâmetros dos nós |

**Ficheiro de parâmetros.** Os parâmetros que costumamos afinar estão em `src/missao/config/missao.yaml`, organizados por nó. Os que não estão lá usam o valor por omissão do código. Depois de editar esse ficheiro é preciso correr `colcon build` outra vez, porque o launch usa a cópia que fica em `install/`. Para testar um cenário diferente sem mexer no original, copia-o e passa a cópia: `ros2 launch missao missao.launch.py parametros:=$PWD/meu_cenario.yaml`. Neste caso não é preciso recompilar. Os argumentos `modo_intercecao` e `busca_automatica` do launch ganham sempre ao YAML.

## Testar a missão

Basta arrancar o launch: o gestor manda o drone procurar assim que o drone e o robô estão prontos. Noutros terminais (com `source install/setup.bash`):

```bash
ros2 topic echo /missao/estado
ros2 topic echo /drone/estado
ros2 topic echo /robo_maritimo/estado
ros2 topic echo /alvo/estimativa
```

O que se deve ver, por ordem:

1. **Gestor:** `ESPERAR -> BUSCA`.
2. **Drone:** descola até 20 m, calcula as faixas ("Busca: 3 faixas espaçadas de 91 m") e procura. Por volta dos 50 s deteta o alvo e passa a `ACOMPANHAR`: fica a ~60 m, virado para ele.
3. **Estimador:** uns segundos depois começa a publicar a posição e a deriva do alvo (~0,5 m/s em +x, a corrente do mundo).
4. **Gestor:** `BUSCA -> RESGATE` e envia a ação `Resgatar` ao robô.
5. **Barco:** `INTERCETAR` ("Interceção daqui a … s em …") → `APROXIMAR` (a menos de 40 m do alvo) → `CONFIRMAR` (a 20 ± 5 m durante 5 s) → `ACOMPANHAR_ALVO`. A ação termina com sucesso.
6. **Hand-off:** o gestor passa a `HAND_OFF` e o drone escreve "Hand-off: … a regressar", volta à base e aterra.
7. **Reboque (fase 4):** ao mesmo tempo, o gestor envia a ação `Rebocar` com a zona de entrega, que por omissão é (15, 10), de onde o robô marítimo sai. O robô faz:
   - `ENGATAR`: vai para o lado do alvo virado para a zona, a 16 m do centro, e roda até ficar de proa para a zona;
   - "Alvo preso com uma barra de … m" e `REBOCAR`: puxa o alvo até o robô chegar a menos de 8 m da zona (na janela do Gazebo vê-se a barra amarela);
   - `ENTREGUE`: fica na zona a segurar o alvo, que fica atrás dele, a favor da corrente.

   O gestor passa a `REBOQUE` quando o drone aterra e a `CONCLUIDA` quando o alvo é entregue. Com `reboque:=false` a missão acaba quando o drone aterra, e o robô continua a acompanhar o alvo com a última deriva estimada.

Para comparar os dois modos do robô:

```bash
ros2 launch missao missao.launch.py headless:=true modo_intercecao:=intercetar
ros2 launch missao missao.launch.py headless:=true modo_intercecao:=perseguir
```

Cada execução grava dois ficheiros em `metricas/`:
- `<data>_<modo>_metricas.csv`: uma linha por segundo, com os estados, as posições, o erro da deteção e da estimativa face à pose real do painel, a distância robô–alvo e a bateria;
- `<data>_<modo>_eventos.csv`: o instante de cada mudança de estado do gestor, do drone e do robô;
- `<data>_<modo>_parametros.yaml`: uma cópia do ficheiro de parâmetros usado, para cada resultado ficar com os parâmetros que o geraram.

**Analisar uma missão.** O `analisar_missao` lê os CSV e escreve um resumo com:
- a linha do tempo;
- as durações (procura, resgate, engate e reboque);
- o erro da deteção e da estimativa;
- a deriva estimada contra a real;
- quantas vezes o drone perdeu o alvo;
- a distância do robô e do alvo à zona de entrega;
- a bateria.

Corre na raiz do repositório:

```bash
ros2 run missao analisar_missao                      # última execução em metricas/
ros2 run missao analisar_missao metricas/<ficheiro>_metricas.csv
ros2 run missao analisar_missao --todas              # tabela com todas as execuções (ex.: intercetar vs perseguir)
```

O resumo aparece no terminal e fica guardado junto aos CSV, em `<execução>_analise.txt`. Com `--todas`, a tabela fica em `metricas/comparacao.txt`.

**Outros testes úteis:**

- **Regra da bateria:** com o drone em `ACOMPANHAR`, força o regresso com `ros2 param set /drone_missao bateria_minima 0.95`. O drone escreve "Bateria a …: a regressar" e o robô continua a missão sozinho.
- **Recuperação (espiral):** para o drone perder o alvo, sobe o limiar do detetor com `ros2 param set /detetor area_min 1000000.0`. O drone passa a `RECUPERAR` e faz uma espiral à volta de onde o alvo deve estar, virado para o centro. Para ver o reencontro, repõe o limiar com `ros2 param set /detetor area_min 30.0`.
- **Arranque manual:** `ros2 launch missao missao.launch.py busca_automatica:=false`, e depois `ros2 service call /missao/iniciar std_srvs/srv/Trigger`.
- **Ver a ação de reboque diretamente**, com o robô já em `ACOMPANHAR_ALVO` e a missão lançada com `reboque:=false`:
  ```bash
  ros2 action send_goal --feedback /robo_maritimo/rebocar missao_interfaces/action/Rebocar "{x: 15.0, y: 10.0}"
  ```
- **Ver a ação de resgate diretamente**, sem gestor:
  ```bash
  ros2 action send_goal --feedback /robo_maritimo/resgatar missao_interfaces/action/Resgatar "{x: -30.0, y: -30.0}"
  ```

Os objetivos manuais da fase 1 continuam disponíveis para testes. Passam à frente da busca e da ação:

```bash
ros2 topic pub --once /drone/objetivo geometry_msgs/msg/PointStamped "{point: {x: 30.0, y: 40.0}}"
ros2 topic pub --once /robo_maritimo/objetivo geometry_msgs/msg/PointStamped "{point: {x: 0.0, y: -20.0}}"
```

Os testes unitários (geometria, regressão, interceção, flake8 e pep257) correm sem simulação:

```bash
colcon test --packages-select missao && colcon test-result --verbose
```

## Resultados (simulação headless, corrente de 0,5 m/s em +x)

| | `intercetar` | `perseguir` |
|---|---|---|
| Primeira deteção (desde o arranque) | 50,9 s | 50,9 s |
| Pedido de resgate enviado | 54,4 s | 54,4 s |
| Barco em `ACOMPANHAR_ALVO` (confirmado) | 81,1 s | 81,5 s |
| Drone em terra (missão `CONCLUIDA`) | 138,9 s | 139,4 s |
| Distância mínima robô–alvo | 14,4 m | 14,5 m |
| Erro médio da deteção / da estimativa | 2,1 m / 1,7 m | 2,1 m / 1,7 m |
| Deriva estimada (real: 0,50 m/s) | 0,51 m/s | 0,51 m/s |

Neste cenário os dois modos quase empatam. O alvo deriva a 0,5 m/s e o robô anda a 2,5 m/s, por isso em ~25 s o alvo só anda ~12 m, e o ponto de interceção fica perto da posição atual. A vantagem da interceção cresce quando a deriva é maior ou quando o alvo se afasta do robô. Para experimentar sem mexer na simulação: `ros2 param set /corrente velocidade 1.5`.

Depois do hand-off o robô acompanha o alvo só com a deriva estimada. A distância real passou de 20 m para 28 m em 7 min, o que dá um erro de deriva de ~0,02 m/s.

O teste da recuperação foi feito "cegando" o detetor durante 90 s com o drone a acompanhar. O drone passou a `RECUPERAR` e fez a espiral à volta do centro, que andou com a deriva. Quando o detetor voltou, o drone reencontrou o alvo e passou a `ACOMPANHAR`.

## Nós

| Nó | Função |
|---|---|
| `poses` | Lê o `dynamic_pose/info` do Gazebo (com `ign topic -e --json-output`) e publica uma `PoseStamped` por entidade |
| `detetor` | Segmenta o painel vermelho em HSV, calcula o centróide e projeta-o na água intersetando o raio da câmara com um plano horizontal. Publica `DetecaoAlvo` |
| `estimativa_deriva` | Ajusta retas x(t) e y(t) às deteções dos últimos 15 s (regressão linear), rejeita outliers e publica a posição e a deriva do alvo em `EstimativaAlvo` |
| `drone_missao` | Máquina de estados do drone: `POUSADO → DESCOLAR → BUSCAR → ACOMPANHAR ⇄ RECUPERAR → REGRESSAR → ATERRAR`, mais `IR` para objetivos manuais. Aplica a regra da bateria em qualquer estado de voo |
| `robo_maritimo_missao` | Servidor das ações `Resgatar` (`AGUARDAR → INTERCETAR → APROXIMAR → CONFIRMAR → ACOMPANHAR_ALVO`) e `Rebocar` (`ENGATAR → REBOCAR → ENTREGUE`), com controlo vetorial de velocidade por `/barco_1/cmd_vel`. `NAVEGAR` para objetivos manuais |
| `gestor_missao` | Arranca a busca, envia a ação `Resgatar` quando há estimativa, trata do hand-off (o drone regressa quando o robô confirma) e publica `/missao/estado` (`ESPERAR → BUSCA → RESGATE → HAND_OFF → REBOQUE → CONCLUIDA`, ou `FALHOU`) |
| `registo_metricas` | Grava os CSV de métricas e de eventos |

## Tópicos

| Tópico | Tipo | Origem | Uso |
|---|---|---|---|
| `/world/oceano_corrente/dynamic_pose/info` | `ignition.msgs.Pose_V` (Gazebo) | Gazebo | lido diretamente pelo nó `poses`; não tem ponte (ver nota abaixo) |
| `/<nome>/pose_mundo` | `geometry_msgs/PoseStamped` | `poses` | pose de `drone`, `robo_maritimo` (`barco_1` no Gazebo), `barco` (`barco_2`) e `marca_vermelha`; `barco` e `marca_vermelha` servem de *ground truth* |
| `/drone/cmd_vel` | `geometry_msgs/Twist` | `drone_missao` | controlador de velocidade do MBZIRC |
| `/drone/objetivo` | `geometry_msgs/PointStamped` | utilizador | coordenada (x, y) a visitar (testes) |
| `/drone/comando` | `std_msgs/String` | `gestor_missao` | `buscar` ou `regressar` |
| `/drone/estado` | `std_msgs/String` | `drone_missao` | estado atual |
| `/barco_1/cmd_vel` | `geometry_msgs/Twist` | `robo_maritimo_missao` | convertido em força dos motores pelo `barco_teleop.py` |
| `/robo_maritimo/objetivo` | `geometry_msgs/PointStamped` | utilizador | coordenada (x, y) a atingir (testes) |
| `/robo_maritimo/resgatar` | ação `missao_interfaces/Resgatar` | servidor: `robo_maritimo_missao`; cliente: `gestor_missao` | objetivo (x, y) do alvo; feedback `distancia` e `estado`; resultado `sucesso` e `mensagem` |
| `/robo_maritimo/estado` | `std_msgs/String` | `robo_maritimo_missao` | estado atual |
| `/alvo/detecao` | `missao_interfaces/DetecaoAlvo` | `detetor` | posição do alvo no mundo e confiança (pela área da mancha) |
| `/robo_maritimo/rebocar` | ação `missao_interfaces/Rebocar` | servidor: `robo_maritimo_missao`; cliente: `gestor_missao` | objetivo: a zona de entrega (x, y); feedback `distancia` (ao ponto de engate, depois do robô à zona) e `estado` |
| `/robo_maritimo/reboque/soltar` | `ignition.msgs.Empty` (Gazebo) | `robo_maritimo_missao` | solta as duas pontas da barra de reboque |
| `/world/oceano_corrente/create`, `/world/oceano_corrente/remove` | serviços do Gazebo | `robo_maritimo_missao` | criam e apagam a barra de reboque com a simulação a correr |
| `/alvo/estimativa` | `missao_interfaces/EstimativaAlvo` | `estimativa_deriva` | posição do alvo prevista para o instante do `header` e deriva (vx, vy) |
| `/missao/estado` | `std_msgs/String` | `gestor_missao` | estado da missão |
| `/missao/iniciar` | serviço `std_srvs/Trigger` | `gestor_missao` | arranca a missão quando `busca_automatica:=false` |
| `/alvo/imagem_depuracao` | `sensor_msgs/Image` | `detetor` | máscara e contorno sobre a imagem; só com `publicar_depuracao:=true` |
| `/model/drone/battery/linear_battery/state` | `sensor_msgs/BatteryState` | ponte no `missao.launch.py` | bateria do drone |
| `/clock` | `rosgraph_msgs/Clock` | ponte do MBZIRC | todos os nós usam `use_sim_time` |

**Porque é que as poses não usam uma ponte.** O `dynamic_pose/info` traz cada pose só com `name`. A ponte do Galactic converte-o em `TFMessage` com o `child_frame_id` vazio, e os nomes perdem-se. O `/drone/pose` do MBZIRC também não serve: só traz as poses dos rotores e sensores relativas ao drone, e não a pose do drone no mundo. Por isso o nó `poses` lê o Gazebo diretamente.

Tópicos do MBZIRC para as próximas fases, confirmados com a simulação a correr:

| Tópico | Tipo | Notas |
|---|---|---|
| `/drone/slot0/image_raw` | `sensor_msgs/Image` | `rgb8`, 1280×960, ~17 Hz (20 Hz × real-time factor) |
| `/drone/slot0/camera_info` | `sensor_msgs/CameraInfo` | fx = fy = 1108,5; cx = 640,5; cy = 480,5; frame `drone/sensor_0/sensor_link/camera` |
| `/drone/slot0/optical/image_raw`, `/drone/slot0/optical/camera_info` | | as mesmas, no frame ótico |
| `/model/drone/battery/linear_battery/state` | `ignition.msgs.BatteryState` → `sensor_msgs/BatteryState` | o MBZIRC não lhe faz ponte; a ponte do Galactic funciona. `percentage` vem de 0 a 100 |
| `/drone/imu/data`, `/drone/magnetic_field`, `/drone/air_pressure` | | o drone não tem GPS nem odometria |
| `/drone/slot1/points`, `/drone/slot1/scan` | | LiDAR 3D |

A câmara fica a (0,1; 0; 0,053) m do centro do drone e aponta para a frente.

No Galactic, o `ros2 topic echo` ainda não tem `--once`. Para ver uma só mensagem: `timeout 5 ros2 topic echo <tópico> | head -40`.

## Parâmetros principais

**`drone_missao`**

| Parâmetro | Omissão |
|---|---|
| `altitude` | 20 m |
| `velocidade_cruzeiro` | 4 m/s |
| `velocidade_vertical` | 1,5 m/s |
| `velocidade_aterrar` | 0,5 m/s |
| `ganho_posicao` | 0,5 |
| `ganho_altitude` | 0,8 |
| `ganho_rumo` | 1,0 |
| `tolerancia_posicao` | 1 m |
| `tempo_pairar` | 5 s |
| `base_x`, `base_y` | 0, 0 |
| `altura_base` | 1 m |
| `cmd_vel_referencial` | `corpo` (alternativa: `mundo`) |
| `busca_automatica` | `false` (quem manda começar é o gestor; a `true` o drone arranca sozinho) |
| `area_x_min`, `area_x_max`, `area_y_min`, `area_y_max` | −120, 120, −120, 120 m |
| `fov_horizontal` | 1,0472 rad (60°, câmara HD) |
| `depressao_min` | 0,2 rad (o alcance útil é altitude/tan(0,2) ≈ 99 m a 20 m) |
| `sobreposicao` | 0,2 |
| `distancia_acompanhamento` | 60 m (com a câmara frontal, o alvo fica na parte de baixo da imagem) |
| `confianca_min` | 0,3 |
| `tempo_perda_alvo` | 3 s (sem deteções, passa a `RECUPERAR`) |
| `raio_espiral_inicial`, `passo_espiral`, `raio_espiral_max` | 60 m, +30 m por volta, 180 m (depois volta a `BUSCAR`) |
| `passo_angular`, `tolerancia_espiral` | 0,3 rad entre pontos, 8 m |
| `bateria_minima`, `margem_bateria` | 0,15 e 0,05 |
| `autonomia` | 1800 s (consumo mínimo assumido; o real é medido a partir da bateria) |

**Espaçamento entre faixas.** A câmara é frontal, por isso vê a água de ~2,3 × altitude até ao alcance útil d = altitude/tan(`depressao_min`). A essa distância a largura vista é 2·d·tan(FOV/2). O espaçamento é essa largura × (1 − `sobreposicao`), o que dá ~91 m a 20 m de altitude. As faixas são paralelas a x, e a busca começa no início de faixa mais próximo.

**Bateria necessária** = (distância à base / `velocidade_cruzeiro` + altura / `velocidade_aterrar`) × consumo por segundo.

**Espiral de recuperação.** O centro é a última estimativa mais a deriva × tempo decorrido, e por isso anda com o alvo. A espiral começa no ângulo onde o drone já está e o raio cresce `passo_espiral` por volta. Como a câmara é frontal, o drone vai sempre virado para o centro. Sem estimativa, usa a última deteção.

**`estimativa_deriva`**

| Parâmetro | Omissão |
|---|---|
| `janela` | 20 s de deteções |
| `pontos_min`, `duracao_min` | 10 pontos e 2 s antes de publicar |
| `distancia_outlier` | 6 m (deteção nova mais longe disto da reta atual é rejeitada) |
| `limite_residuo` | 3 m (na segunda passagem do ajuste tiram-se os pontos mais longe disto) |
| `rejeicoes_para_recomecar` | 15 rejeições seguidas recomeçam o ajuste |
| `validade` | 5 s sem deteções e deixa de publicar |

A estimativa publicada é a posição prevista para o instante do `header`. Quem a usa (robô, drone, métricas) leva-a até ao instante atual com a deriva.

**`detetor`**

| Parâmetro | Omissão |
|---|---|
| `matiz_baixa_max`, `matiz_alta_min` | 10, 170 (as duas gamas do vermelho, matiz 0–180) |
| `saturacao_min`, `valor_min` | 100, 50 |
| `nucleo_morfologia` | 5 px |
| `area_min` | 30 px |
| `area_confianca_total` | 400 px (área a partir da qual a confiança é 1) |
| `camera_xyz`, `camera_rpy` | (0,1; 0; 0,053), (0; 0; 0): pose da câmara do `slot0` no drone |
| `altura_plano` | 1,2 m, a altura do painel acima da água (ver validação abaixo) |
| `distancia_max` | 300 m |
| `atraso_pose_max` | 0,1 s (diferença máxima entre a imagem e a pose do drone usada) |

**Validação** (drone a 20 m de altitude, câmara frontal, erro contra a pose real da `marca_vermelha`):

| Distância ao alvo | Erro com `altura_plano` = 0 | Erro com `altura_plano` = 1,2 |
|---|---|---|
| 40–60 m | 4,8 m | 1,6 m |
| 60–80 m | 6,3 m | 1,7 m |
| 80–100 m | 8,0 m | 2,0 m |

Quase todo o erro está ao longo da linha de visão. O erro lateral fica abaixo de 0,6 m. O painel está a ~1,2 m acima da água, e com a câmara oblíqua isso faz o alvo parecer mais longe se se projetar em z = 0. O alvo é detetado de forma contínua a partir de ~110 m.

Na fase 2 usava-se `altura_plano` = 0. Na fase 3 o CSV mostrou que esse erro, além de grande, muda com a distância. Enquanto o drone se aproxima, isso parece movimento do alvo: o estimador dava 0,58–0,9 m/s de deriva, quando a real é 0,50 m/s. Depois do hand-off o robô afastava-se do alvo ~18 m em 4 min. Com 1,2 m o erro médio da deteção desce para ~2 m e a deriva estimada fica em 0,50–0,54 m/s.

**`robo_maritimo_missao`**

| Parâmetro | Omissão |
|---|---|
| `velocidade_max` | 2,5 m/s |
| `velocidade_max_perto` | 1,5 m/s a partir de `APROXIMAR` (para não chegar lançado e ficar perto demais) |
| `ganho_velocidade` | 0,15 (velocidade desejada por metro de distância) |
| `ganho_correcao` | 1,0 (peso do erro entre a velocidade desejada e a medida) |
| `filtro_velocidade` | 0,2 (filtro passa-baixo da velocidade medida a partir das poses) |
| `ganho_rumo` | 1,0 |
| `rotacao_max` | 0,8 rad/s |
| `erro_rumo_max` | 0,8 rad (acima disto só roda, sem avançar) |
| `velocidade_minima` | 0,2 m/s (abaixo disto não comanda nada) |
| `modo_intercecao` | `intercetar` ou `perseguir` |
| `distancia_seguranca` | 20 m do centro do alvo (o navio tem 22 m) |
| `distancia_aproximar` | 40 m (abaixo disto passa de `INTERCETAR` a `APROXIMAR`) |
| `tolerancia_chegada` | 5 m (em torno dos 20 m de distância de segurança) |
| `histerese` | 5 m |
| `tempo_confirmar` | 5 s junto ao alvo para confirmar |

O casco do WAM-V escorrega muito de lado. Por isso o robô não aponta a proa para o objetivo: aponta-a para **a = v_d + k·(v_d − v)**, onde v_d é a velocidade desejada em direção ao objetivo e v é a velocidade medida. Assim a derrapagem lateral e a corrente são anuladas. Com o controlo de rumo simples, o robô ficava em órbita a ~10 m do ponto.

**Interceção.** Com o alvo em p, a deriva v, o robô em b e a velocidade do robô s, resolve-se |p + v·t − b| = s·t, que é uma equação do 2.º grau em t, e escolhe-se o menor t > 0. O robô vai para o ponto a 20 m de p + v·t. Em modo `perseguir` vai sempre para o ponto a 20 m de p. Se não houver solução (alvo mais rápido do que o robô), persegue. De `APROXIMAR` em diante, a velocidade desejada soma a deriva do alvo (v + k·(objetivo − posição)) para o robô andar com ele em vez de ficar para trás.

Depois do hand-off já não há deteções. O robô acompanha o alvo extrapolando a última estimativa com a deriva.

## Reboque (fase 4)

**Como o alvo fica preso.** Quando está alinhado, o robô cria com a simulação a correr um modelo novo, `barra_reboque`, através do serviço `/world/oceano_corrente/create`. É uma barra amarela com uma articulação vertical em cada ponta. Nas pontas, o plugin `DetachableJoint` prende uma ao `base_link` do robô (`barco_1`) e a outra ao `link` do robô (`barco_2`). É o mesmo plugin que o mundo já usa para prender o painel vermelho ao robô. O SDF é gerado em `gazebo.py` e enviado com o `ign` da linha de comandos. Nenhum ficheiro da simulação é alterado. Para soltar, publica-se no tópico `/robo_maritimo/reboque/soltar` e apaga-se a barra.

**Porque é uma barra articulada e não uma junta direta.** A primeira versão prendia o barco à deriva diretamente ao robô com uma junta fixa, acrescentada pelo serviço `entity/system/add`. Funcionava, mas o conjunto ficava rígido. A única forma de o virar era a diferença de força entre os dois motores (~1900 N·m), e a corrente a empurrar o navio de lado, com 16 m de braço, fazia mais do que isso. O conjunto acabava alinhado com a corrente, como um cata-vento, e era arrastado: em duas de três tentativas o alvo acabou mais longe da zona do que estava. Com as articulações, cada barco roda à vontade em relação à barra, como num reboque com cabo, e o robô consegue sempre apontar para onde quer ir.

**Engate.** O robô vai para o lado do alvo virado para a zona de entrega, a 16 m do centro, ou seja, 11 m de meio navio + 2,5 m de meio WAM-V + folga, e roda até ficar de proa para a zona. Só então cria a barra, com o comprimento da distância nesse momento. Assim começa logo a puxar na direção certa.

**Força.** O barco à deriva pesa 3000 kg e tem muito arrasto (~2500 N a 1 m/s em relação à água). Os dois motores do WAM-V dão no máximo ~1440 N. O `barco_teleop.py` está afinado para o WAM-V sozinho: com 2,5 m/s de `cmd_vel` pede só ~580 N. Por isso, a rebocar, o robô pede até 4 m/s (`avanco_max_reboque`), que já é perto da força máxima. Além disso, puxa mesmo com o rumo um pouco torto, porque se parar para virar a corrente leva o conjunto.

**Entrega.** Com a barra articulada, o alvo pode estar em qualquer ângulo atrás do robô, e o robô não tem sensor para o ver. Por isso a entrega mede-se pelo robô: quando chega a menos de `raio_entrega` da zona, a ação termina com sucesso. O robô fica a segurar a posição, com o alvo preso atrás dele, a favor da corrente, como um barco amarrado a uma boia. Para segurar o navio contra 0,5 m/s de corrente são precisos ~870 N, dentro do que os motores dão. Com `largar_na_entrega: true`, o robô solta o alvo, mas então a corrente volta a levá-lo.

| Parâmetro (`robo_maritimo_missao`) | Omissão |
|---|---|
| `distancia_engate` | 16 m |
| `tolerancia_engate`, `erro_rumo_engate`, `tempo_engate` | 3 m, 0,25 rad, 2 s |
| `velocidade_reboque` | 0,8 m/s |
| `avanco_max_reboque` | 4,0 ("m/s" pedidos ao `barco_teleop`, ~força máxima) |
| `ganho_correcao_reboque`, `erro_rumo_max_reboque`, `rotacao_max_reboque` | 1,0; 1,2 rad; 2,0 rad/s |
| `escala_forca_reboque` | 0,5 m/s (a segurar, a força baixa quando o vetor de velocidade pedido é menor do que isto) |
| `raio_entrega` | 8 m |
| `largar_na_entrega` | `false` |
| `mundo`, `link_robo`, `modelo_alvo`, `link_alvo` | `oceano_corrente`, `base_link`, `barco_2`, `link` |
| `nome`, `modelo_gazebo`, `topico_cmd_vel` | `robo_maritimo`, `barco_1`, `/barco_1/cmd_vel` |

**Resultado (simulação headless, missão completa com reboque):**

| Evento | Tempo |
|---|---|
| Barco confirma o alvo; começa o `ENGATAR` | 81 s |
| Barra criada (18,2 m); começa o `REBOCAR` | 105 s |
| Barco na zona de entrega (`ENTREGUE`) | 120 s |
| Drone em terra; missão `CONCLUIDA` | 139 s |

A barra manteve-se entre 18,2 e 19,3 m durante todo o reboque. A segurar, ao longo de 330 s, o robô ficou em média a 3,2 m da zona (no máximo 7,8 m), com o alvo atrás dele, em média a 22,3 m da zona e a favor da corrente.

A zona de entrega (`zona_entrega_x`, `zona_entrega_y`) é um parâmetro do `gestor_missao`. Está no `config/missao.yaml`, na secção `/**`, para o `registo_metricas` usar o mesmo valor. O CSV ganhou a coluna `distancia_alvo_entrega`, com a distância real do alvo à zona.

## Referências

Projetos recomendados nas aulas que serviram de referência, sem serem instalados: o ambiente está fixo em Galactic + Fortress + MBZIRC e estes pedem outras versões.

- **VRX (Virtual RobotX), da OSRF:** o robô marítimo é o WAM-V do MBZIRC (`mbzirc_usv_base`), que vem do VRX. As tarefas de *station keeping* e *wayfinding* do VRX são parecidas com o nosso `ACOMPANHAR_ALVO` e `INTERCETAR`.
- **Tutoriais marítimos do Gazebo (GZ Maritime):** flutuação, hidrodinâmica, propulsores e ondas. São os plugins que o mundo usa, e explicam a derrapagem lateral do WAM-V e a força precisa para rebocar.
- **Tutoriais do Gazebo** ("Understanding the GUI", "Manipulating Models"): para a demonstração com a janela do Gazebo.

## Notas de execução

- **Bateria:** a descarga é linear no tempo de simulação, 100 % em 30 min (`flight_time` do `drone.yaml`), e não depende da velocidade. Continua a descarregar com o drone pousado.
- **Real-time factor:** é de ~0,85–0,9 em `headless`. Os tempos dos nós são de simulação (`use_sim_time`).
- **Placa NVIDIA:** mesmo com a box criada com `--nvidia`, o Gazebo renderiza com a placa integrada (o `nvidia-smi` não mostra processos). Ainda não foi investigado.
- **Frente do robô marítimo (`barco_1`):** é o +x do modelo, confirmado pelo rumo durante a navegação.
