# Spec — Calibração de DVL no Kalibr (`kalibr_calibrate_dvl`)

> **O quê e por quê.** Fonte da verdade da intenção. Sem decisão técnica nem passos de implementação
> (esses vão em `decisions.md` e `plan.md`).

## Problema

O projeto de SLAM subaquático tem 4 sensores num cilindro: câmera estéreo ZED 2i, IMU interna da ZED,
**DVL Water Linked A50** e **IMU externa Microstrain**. A câmera estéreo e o par câmera-IMU (com a
Microstrain como IMU de referência) já foram calibrados **submersos** pelo Kalibr. Falta calibrar o
**DVL**: sua transformação rígida em relação à IMU de referência, o offset temporal entre os relógios
e a escala de velocidade (erro de velocidade do som).

O Kalibr — calibrador offline consagrado — **não tem conceito de DVL**. O AQUA-SLAM tem um modelo de
DVL, mas o calibra **online, dentro do factor graph do SLAM** (g2o), o que exige rodar o SLAM inteiro
e não produz uma calibração offline reutilizável e auditável. Queremos a calibração do DVL **como uma
etapa offline do Kalibr**, unindo o rigor batch/tempo-contínuo do Kalibr ao modelo de medição do DVL
inspirado no AQUA-SLAM.

## Contexto

- **Kalibr** (ver `.ai/docs/architecture.md`, `.ai/docs/api.md`, `.ai/docs/domain/cam-imu-calibration.md`):
  estimação batch em tempo contínuo; a trajetória do corpo é uma **B-spline de pose** no frame da IMU
  de referência; termos de erro de reprojeção/acelerômetro/giroscópio no `aslam_backend`. Pipeline em
  Python (`kalibr_imu_camera_calibration/IccSensors.py`, `IccCalibrator.py`), error terms de IMU em C++
  (`kalibr_errorterms`).
- **DVL A50 / driver `paagutie/dvl-a50`** (msgs `dvl_msgs`): tópico `/dvl/data` (msg `DVL`) traz
  `velocity` (Vector3 vx/vy/vz, m/s), `covariance` (3×3 (m/s)²), `fom`, `beams[]` (velocidade por-feixe,
  `distance`, `valid`), `altitude`, `velocity_valid`, `time_of_validity` (µs). Tópico `/dvl/position`
  (msg `DVLDR`) traz dead-reckoning (não usado aqui).
- **Frame de referência:** a **Microstrain** foi a `--imu` #1 na calibração multi-IMU/câmera existente,
  então o extrínseco do DVL é estimado **diretamente no frame da Microstrain** (`T_dvl_imu`), sem composição.
- **Calibração cam-IMU existente (submersa)** está disponível como insumo (intrínsecos da câmera,
  extrínsecos câmera-IMU, ruído da IMU) e é reutilizada. Fluxo já praticado pelo usuário: (1) calibração
  estéreo submersa a partir de um bag só de imagens; (2) calibração câmera-IMU submersa a partir de um
  bag com imagens retificadas + IMU.
- **A B-spline de pose depende do bag em questão.** O DVL não observa o alvo (só dá velocidade), então a
  trajetória precisa ser reconstruída a partir de câmera+IMU **do próprio bag do DVL** — não se reaproveita
  a trajetória de um bag anterior, apenas os **parâmetros** cam-IMU já calibrados. Logo, o bag de DVL
  precisa conter **estéreo retificado + IMU + DVL** juntos.
- **Coleta:** tanque com o AprilGrid do Kalibr (6×6, `tagSize=0.088`, `tagSpacing=0.3`), submerso, com
  todo o equipamento submerso. É possível capturar, ao mesmo tempo, a câmera vendo o alvo submerso e o
  DVL com bottom-lock.
- **Origem dos dados / ROS 2 → ROS 1:** os bags são gravados pelo projeto do usuário em **ROS 2** e
  convertidos para **ROS 1** para o Kalibr (que é ROS 1). A mensagem `dvl_msgs/DVL` é custom; a via de
  menor atrito é **extrair o stream do DVL para CSV** (no estilo do `imu0.csv` do Kalibr) e consumir por
  CSV, evitando regerar a mensagem custom em ROS 1 (detalhe em `decisions.md`).
- **Física da observação:** o DVL mede velocidade linear; a rotação do extrínseco é observável pela
  direção da velocidade, a translação (braço de alavanca) só sob excitação rotacional (`v_dvl = v_imu
  + ω × r`), e a escala de velocidade é observável porque a **escala métrica da trajetória é fixada
  pelo alvo** de tamanho conhecido.

## Requisitos

- **R1 — Calibração offline do DVL no Kalibr.** A partir de um ROS bag com câmera + IMU + DVL (e a
  calibração cam-IMU existente), estimar offline: `T_dvl_imu` (rotação + translação), o offset temporal
  DVL↔IMU e a escala de velocidade `s` do DVL.
- **R2 — Modelo de medição por velocidade 3D.** Usar a medição `velocity` (vx/vy/vz) do DVL como
  observação, comparada à velocidade predita pela derivada da B-spline de pose transformada por
  `T_dvl_imu` e escalada por `s`.
- **R3 — Ponderação estatística.** Ponderar (whitening) cada resíduo pela `covariance` reportada pelo
  DVL (com fallback razoável quando ausente/inválida).
- **R4 — Gating de dados.** Descartar amostras sem bottom-lock/inválidas usando `velocity_valid`,
  `fom`, `altitude` e a validade dos feixes.
- **R5 — Timestamp preciso.** Usar `time_of_validity` do DVL como timestamp da amostra, e estimar o
  offset temporal residual DVL↔IMU dentro do mesmo mecanismo contínuo que o Kalibr usa para cam-IMU.
- **R6 — Desenvolvido dentro do Kalibr, seguindo sua organização.** A funcionalidade é adicionada **ao
  próprio pacote `kalibr`** (não um pacote separado), numa **branch de feature `calib_cam_imu_dvl`**,
  para que o Kalibr concentre toda a calibração. Reusa a maquinaria do Kalibr (B-spline `aslam_splines`,
  `aslam_backend`, `ConfigReader`, leitores de dataset) e **porta do AQUA-SLAM o modelo de medição do
  DVL** (`dvl_model/dvl.cpp` e a pré-integração DVL/giroscópio) como referência de física, reimplementado
  no estilo Kalibr. Aceita-se conscientemente que isso diverge do upstream `ethz-asl/kalibr` (já é um fork).
- **R7 — Interface de linha de comando.** Uma CLI no estilo Kalibr (`kalibr_calibrate_dvl`, ou uma
  extensão de `kalibr_calibrate_imu_camera` com `--dvl` — decisão em `decisions.md`), com flags no padrão
  do Kalibr (`--bag`, `--cams`, `--imu`, a calibração cam-IMU de referência, a fonte do DVL, `--target`…).
- **R8 — Configuração do DVL.** Ler um YAML de configuração do DVL (tópico, ruído/escala inicial,
  extrínseco inicial a partir do xacro) no estilo dos YAMLs de câmera/IMU do Kalibr.
- **R9 — Saída no formato Kalibr.** Emitir um YAML de resultado no estilo Kalibr contendo `T_dvl_imu`
  (matriz 4×4), `timeshift_dvl_imu` (s) e `velocity_scale` (adimensional), mais um relatório com
  resíduos, e um resumo textual — coerente com `*-results-imucam.txt`/`*-camchain-imucam.yaml`.
- **R10 — Diagnóstico de qualidade.** Reportar métricas que permitam julgar a calibração: RMS do
  resíduo de velocidade (m/s), número de amostras usadas/descartadas, escala estimada, offset
  temporal estimado, e (quando possível) recuperação de covariância dos parâmetros.
- **R11 — Fluxo unificado num bag combinado.** A calibração roda num único comando sobre um **bag
  combinado** (estéreo retificado + IMU + DVL). Os intrínsecos da câmera vêm da calibração estéreo
  prévia (pré-condição). Dois modos quanto ao cam-IMU:
  - **Modo A (padrão) — cam-IMU fixo:** reutiliza a calibração câmera-IMU submersa existente como
    **fixa**; reconstrói a B-spline a partir do bag e estima **só** os parâmetros do DVL. Mais robusto.
  - **Modo B (opcional, via flag) — conjunto:** co-otimiza câmera-IMU-DVL no mesmo batch.
- **R12 — Ingestão do DVL por CSV (e/ou tópico).** O leitor de dataset do DVL consome o stream do DVL
  a partir de um **CSV** (timestamp + velocidade 3D + covariância + validade/fom/altitude), no estilo do
  `imu0.csv`, para contornar o atrito ROS 2 → ROS 1 com a mensagem custom `dvl_msgs/DVL`. (Ler direto de
  um tópico ROS 1, caso `dvl_msgs` seja regerado, pode ser suportado como alternativa.)

## Não-objetivos

- **Calibrar a orientação dos feixes (alpha/beta) do DVL.** Fica de fora nesta versão (usaremos a
  velocidade 3D agregada). O design não deve impedir uma extensão futura, mas não a implementa.
- **Recalibrar câmera/IMU por padrão.** No Modo A (padrão), intrínsecos, extrínsecos câmera-IMU e ruído
  da IMU vêm da calibração existente e são mantidos **fixos** (reuso). A co-otimização conjunta (Modo B)
  é um caminho opcional, não o comportamento padrão.
- **Calibrar intrínsecos da câmera.** Continuam vindo da etapa de calibração estéreo prévia; não são
  estimados aqui.
- **Emitir formato AQUA-SLAM.** Conversão para o YAML OpenCV do AQUA-SLAM (`T_dvl_c`, alpha/beta) fica
  para trabalho futuro; esta versão emite só formato Kalibr.
- **Fusão/estimativa online.** Não é objetivo rodar SLAM nem calibração em tempo real (isso é o papel
  do AQUA-SLAM). Esta é uma ferramenta offline.
- **Usar o dead-reckoning do DVL (`/dvl/position`).** Não é insumo de calibração.
- **Rolling shutter, multi-DVL, DVL sem bottom-lock (water-track).** Fora de escopo.
- **Modelar refração/porta da câmera.** Já tratado na calibração cam-IMU submersa existente.

## Critérios de aceitação

- [ ] **CA1:** Existe uma CLI que, dado um bag com câmera+IMU+DVL, a calibração cam-IMU de referência e
      um YAML de config do DVL, roda a calibração offline e termina produzindo os arquivos de saída.
- [ ] **CA2:** A saída contém `T_dvl_imu` (4×4), `timeshift_dvl_imu` (s) e `velocity_scale`, em YAML no
      estilo Kalibr, mais um relatório de resíduos e um resumo textual.
- [ ] **CA3:** A ferramenta descarta amostras de DVL sem bottom-lock/inválidas e ponderas os resíduos
      pela covariância reportada (verificável em log/relatório: nº usados vs. descartados).
- [ ] **CA4:** A funcionalidade é desenvolvida na branch `calib_cam_imu_dvl` **dentro do pacote
      `kalibr`**, seguindo a organização do projeto (error term em `src/`, sensor Python em
      `kalibr_imu_camera_calibration/`, CLI em `python/`); o workspace compila com `catkin build`.
- [ ] **CA5 (validação física):** Num dataset de tanque com excitação em 6-DOF, a `velocity_scale`
      estimada fica próxima do esperado para o `sound_speed` configurado, o RMS do resíduo de velocidade
      fica em faixa aceitável (a definir com dados reais), e a translação de `T_dvl_imu` é coerente com
      a montagem do xacro (dentro de uma tolerância a definir).
- [x] **CA6:** O termo de erro de DVL tem teste unitário (à moda do `TestErrorTerms.cpp`) validando o
      resíduo e (se aplicável) as jacobianas. ✓ GTest `testDvlVelocity` (jacobianas via harness) + teste
      de recuperação Python (`test/dvl/test_dvl_recovery.py`) recuperam extrínseco e escala exatamente.
- [ ] **CA7:** Há documentação de uso (procedimento de coleta em tanque + comandos), coerente com
      `scripts/docs/kalibr.md`.

## Perguntas em aberto

- **NEEDS CLARIFICATION (dados):** No tanque, o DVL mantém bottom-lock (superfície refletora dentro do
  alcance, ~0.05–50 m) **enquanto** a câmera vê o alvo? Confirmar a geometria de captura (DVL apontando
  ao fundo/parede; alvo no campo da câmera) — é pré-condição de CA5.
- **NEEDS CLARIFICATION (escala/sound-speed):** Qual `sound_speed` está configurado no A50 durante a
  captura (padrão 1500 m/s ou ajustado)? Define o valor esperado de `velocity_scale` para CA5.
- **NEEDS CLARIFICATION (excitação):** O protocolo de captura garante movimento translacional **e**
  rotacional suficiente para observabilidade do braço de alavanca? (a "dança" de calibração).
- **RESOLVIDO (ROS):** O ambiente do Kalibr é ROS 1 e o driver do DVL é ROS 2. Decidido: **CSV** como
  via padrão de ingestão do DVL (R12, D5), aprovado pelo usuário. Regerar `dvl_msgs` em ROS 1 fica como
  alternativa opcional. Schema do CSV a fixar na tarefa 3.1 do plano.
- **NEEDS CLARIFICATION (tolerâncias):** Faixas numéricas de aceitação (RMS de velocidade, tolerância
  de translação vs. xacro) a fixar após ver o primeiro dataset real (tarefa 6.2 do plano).

## Estado de implementação

> Progresso detalhado (por micro-tarefa) em `plan.md`. Resumo (todas as verificações rodam no container
> `kalibr` via `docker run -v <repo>:/catkin_ws/src/kalibr`, ver D10):

- **Fase 0–2 ✅** — motor matemático, validado em dados sintéticos:
  - Branch `calib_cam_imu_dvl`; build/testes de base OK no container.
  - Spikes: geometria do resíduo em Python puro; escala exige thin C++ (D4/D7).
  - `DvlError.py` (resíduo Python) + `DvlVelocityError` (C++, escala) + GTest `testDvlVelocity`.
  - Gerador sintético + teste de recuperação (TDD) recuperam `T_dvl_imu` e escala **exatamente**.
- **Fase 3 (config/ingestão) — parcial:**
  - ✅ 3.1 schema do CSV + `scripts/config/dvl0.yaml` + `dvl0_example.csv`.
  - ✅ 3.2 `DvlParameters` (round-trip idempotente).
  - ✅ 3.3 `CsvDvlDatasetReader` (gating D7; integra com o termo de erro).
  - ⏸️ 3.4 extrator ROS 2 — **adiada** (feita ao coletar dados reais; independe do núcleo).
- **Fase 4 (`IccDvl` + `IccCalibrator`) ✅:** classe `IccDvl` (4.1), `addVelocityErrorTerms` com recuperação
  exata via IccDvl (4.2), `findTimeOffsetPrior` (4.3), e `IccCalibrator` estendido (4.4: `DvlList`, Modo A/B,
  `fixCamImuDesignVariables`, `saveDvlParametersYaml`).
- **Fase 5 (CLI + saída) ✅:** `kalibr_calibrate_dvl` (5.1, com wiring D12 `reuseProvidedCamImuExtrinsics`),
  `saveDvlResultTxt` + estatísticas de resíduo (5.2), gráfico `plotDvlVelocityError` no relatório (5.3).
- **Fase 6 — parcial:** ✅ 6.1 guia `scripts/docs/kalibr_dvl.md`; ✅ 6.3 `.ai/docs/` atualizados.
  ⏸️ **6.2 validação numérica no tanque** (depende de bag real) e ⏸️ **3.4 extrator ROS 2** (ambiente ROS 2).
- **Critérios:** **CA1–CA4, CA6, CA7 ✅**; **CA5 (validação física) pendente da 6.2** (dados reais).

> **Testes da feature** (rodáveis no container): `test/dvl/*.py` — `dvl_synthetic`, `test_dvl_recovery`,
> `test_dvl_config`, `test_dvl_csv_reader`, `test_iccdvl`, `test_iccdvl_recovery`, `test_iccdvl_timeoffset`;
> além do GTest C++ `testDvlVelocity`. Todos verdes.
