# Decisões — Validação do fluxo de calibração no tanque

> ADR leve. Registra o **porquê** técnico. Durável.

### D1 — Conversão ROS 2 → ROS 1 só dos tópicos de tipo padrão (imagem + IMU)

- **Contexto:** o bag tem tipos custom (`dvl_msgs/DVL`, `petro_interfaces/PressureDepthSensor`) que não têm
  definição em ROS 1; o Kalibr só precisa de imagem + IMU. O DVL entra por CSV (não pelo bag ROS 1).
- **Decisão:** converter (rosbags-convert) apenas os tópicos de imagem estéreo + IMUs, **excluindo**
  `/dvl/data` e `/lar/bar/depth`.
- **Por quê:** evita falha de conversão por tipo custom; reduz o tamanho do bag ROS 1.
- **Consequências:** o CSV do DVL é gerado à parte pelo `ros2_dvl_to_csv.py`.

### D2 — Câmera nas imagens raw + modelo de distorção (sem retificar)

- **Contexto:** o usuário tem intrínsecos + distorção; as imagens são raw color.
- **Decisão:** montar um `camchain.yaml` (2 câmeras) com intrínsecos + `distortion_coeffs` sobre os tópicos
  raw; o Kalibr aplica a distorção internamente. Não gerar imagens retificadas.
- **Por quê:** é o fluxo nativo do Kalibr; retificar adicionaria uma etapa propensa a erro sem ganho.
- **Consequências:** confirmar o `distortion_model` (radtan/equidistant) e a baseline (`T_cn_cnm1`).

### D3 — Ruído da IMU provisório (datasheet), Allan variance adiada

- **Contexto:** não há Allan variance e há pressa.
- **Decisão:** usar valores de datasheet/típicos (possivelmente inflados ~5–10×) no `imu.yaml`. A estimação
  de ruído (Allan) é **spec futura**.
- **Por quê:** destrava a calibração agora; datasheet é funcional (afeta qualidade, não viabilidade).
- **Consequências:** a qualidade do cam-IMU pode melhorar depois com Allan; registrar isso nos resultados.

### D4 — Calibração por bag (independente) + DVL em Modo A; validação por consistência cross-bag

- **Contexto:** o usuário escolheu comparar os extrínsecos entre os 4 bags (repetibilidade).
- **Decisão:** calibrar cam-IMU e DVL em cada bag separadamente; o DVL em **Modo A** reusa o cam-IMU do
  **mesmo** bag. Validar comparando os 4 resultados.
- **Por quê:** consistência entre gravações é o melhor teste de sanidade sem ground-truth estrito.
- **Consequências:** 4 execuções de cada etapa; um script de comparação agrega os resultados.

### D5 — Duas IMUs no cam-IMU (Microstrain referência + ZED), com fallback

- **Decisão:** rodar o `kalibr_calibrate_imu_camera` com `--imu microstrain.yaml zed.yaml` (Microstrain #1).
  Se a ZED IMU degradar a convergência, cair para Microstrain-só.
- **Por quê:** aproveita as duas IMUs (dá `T_zed_micro`), mas não bloqueia se a ZED atrapalhar.
- **Consequências:** o extrínseco do DVL é sempre relativo à Microstrain (referência), independente disso.

### D8 — Rebasear timestamps (Unix epoch → ~0) na conversão ROS 2→ROS 1 e no CSV do DVL

- **Contexto:** o `kalibr_calibrate_imu_camera` falhou com "Spline Coefficient Buffer Exceeded"
  (`[1.78535e9 <= 1.78535e9 < 1.78535e9]`). Causa: timestamps Unix (~1.785e9 s); a B-spline usa float32
  no buffer de tempo, e nessa magnitude o float32 tem ~212 s de resolução — os tempos colapsam.
- **Decisão:** subtrair um `t0` comum (≈ início do bag) de **todos** os timestamps ao gerar o bag ROS 1
  **e** ao extrair o CSV do DVL, mantendo o timing relativo e o alinhamento entre eles. Scripts
  `ros2_to_ros1_kalibr.py` (`--zero-start`, imprime `T0_NS`) e `ros2_dvl_to_csv.py` (`--t0-ns`).
- **Por quê:** fix padrão da comunidade para bags rosbag2 no Kalibr; barato (re-converter ~1 min).
- **Consequências:** o `t0` usado no bag e no CSV do DVL **deve ser o mesmo**. O timeshift/offset
  estimados ficam em relação ao tempo rebaseado (irrelevante — são relativos).

### D7 — Reusar intrínsecos da ZED como radtan 4-params (dropar k3); testar Kalibr depois

- **Contexto:** a calibração da ZED (`zed_opencv_calibration.yaml`) é radtan de **5 params**
  (`k1,k2,p1,p2,k3`) com k3 grande (−2.64 esq / −4.12 dir); o modelo `radtan` do Kalibr é de **4 params**.
- **Decisão:** por escolha do usuário (pressa), montar o `camchain.yaml` com `fx,fy,cx,cy` + `[k1,k2,p1,p2]`
  da ZED, **descartando o k3**. Baseline/`T_cn_cnm1` do campo `T` (mm→m).
- **Por quê:** destrava a validação já; evita recalibrar intrínsecos agora.
- **Consequências / a testar depois:** o k3 grande pode **elevar o erro de reprojeção** e enviesar o
  cam-IMU. **TODO (futuro):** calibrar os intrínsecos com o Kalibr no bag (pinhole-radtan e/ou pinhole-equi)
  e comparar reprojeção vs. a ZED-4params. Registrar na consolidação.

### D9 — Descartar o bag 01: o offset temporal câmera-IMU só assenta ~15 min após ligar

- **Contexto:** rodados os 4 bags, o `timeshift_cam_imu` caiu monotonicamente com o horário de gravação:
  16.78 (18:37) → 9.07 (18:44) → 5.96 (18:51) → 4.29 ms (19:00). Um ajuste
  `ts = 3.40 + 13.37·exp(-t/515 s)` fecha com RMS 0.003 ms (ressalva: 4 pontos, 3 parâmetros — o
  decaimento monotônico é o fato; a forma exata é sugestiva).
- **Decisão:** tratar o **bag 01 como outlier** (sistema "frio", offset 5× acima do assentado) e
  consolidar sobre os bags 02–04. Em capturas futuras, **aguardar ~15–20 min** (≈3 constantes de tempo)
  entre ligar o sistema e gravar.
- **Por quê:** excluir o bag 01 derruba a dispersão da translação em x de **2.1 cm para 0.7 cm** — a maior
  parte da "não-repetibilidade" era esse transitório, não falta de excitação. Um offset temporal errado é
  absorvido pela translação (a rotação é insensível), que é justamente o parâmetro sob suspeita.
- **Consequências:** a ZED aparenta usar timestamping por software com latência que assenta; sem
  sincronização por hardware, cada bag tem seu próprio alinhamento. Escolher o bag também pelo relógio
  assentado, não só pelo resíduo. Vale reavaliar se o `timeshift` deve entrar no AQUA-SLAM (hoje sem campo).

### D10 — O xacro erra a POSIÇÃO da Microstrain (~13 cm), não a orientação

- **Contexto:** a nota inicial do bag 01 (`data/output/piscina_calib_01-analise-camimu.txt`) concluiu que o
  xacro estava impreciso na **orientação** da Microstrain (~43°). **Essa conclusão estava errada:** comparava
  `T_ic` (frame da IMU) com um vetor em `base_link` sem passar pela convenção de *optical frame* da ZED
  (x=direita, y=baixo, z=frente) nem pela geometria interna do `zed_macro`.
- **Decisão:** registrar que (a) a **orientação confere** — 1.52° ± 0.22° entre os 4 bags, compatível com
  `rpy=0 0 0` e com tolerância de fabricação; (b) a **posição não confere** — viés de 13.5 cm com dispersão
  de 1.7 cm entre os bags 02–04 (razão 7.7×), dominado por **x (~12–13 cm)**.
- **Por quê:** o viés é 7.7× a incerteza da medida — os bags discordam entre si muito menos do que todos
  discordam do desenho. É a assinatura de erro de cota, não de ruído de estimação.
- **Consequências:** conferir no CAD a cota da Microstrain (o xacro a põe em `x=-0.09439`; os dados apontam
  para `x≈+0.02`) e o ponto que `zed_node_camera_link` representa. **A calibração mede a posição relativa**,
  então não distingue qual dos dois corpos está fora do lugar. Impacta o DVL: o lever-arm nominal
  Microstrain→DVL do xacro (`[0.094, 0, -0.129]`) herda o mesmo erro e vale como referência frouxa.

### D6 — Consolidação: reportar os 4, escolher por resíduo/consistência

- **Decisão:** apresentar os resultados dos 4 bags (média ± desvio) e escolher a calibração final como o
  bag de melhor qualidade (menor reprojeção/RMS, boa excitação) OU a mediana dos bags consistentes.
- **Por quê:** decisão informada pelos números reais; evita fixar critério antes de ver os dados.
- **Consequências:** as tolerâncias de aceitação são fixadas após a primeira rodada (CA4/CA5).
