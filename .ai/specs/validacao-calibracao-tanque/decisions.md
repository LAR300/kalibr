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

### D6 — Consolidação: reportar os 4, escolher por resíduo/consistência

- **Decisão:** apresentar os resultados dos 4 bags (média ± desvio) e escolher a calibração final como o
  bag de melhor qualidade (menor reprojeção/RMS, boa excitação) OU a mediana dos bags consistentes.
- **Por quê:** decisão informada pelos números reais; evita fixar critério antes de ver os dados.
- **Consequências:** as tolerâncias de aceitação são fixadas após a primeira rodada (CA4/CA5).
