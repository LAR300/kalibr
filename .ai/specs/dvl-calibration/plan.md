# Plano — Calibração de DVL no Kalibr (`kalibr_calibrate_dvl`)

> Micro-tarefas pequenas e verificáveis. O "porquê" das escolhas está em `decisions.md` (D1–D10);
> os requisitos em `spec.md` (R1–R12). Marque `[x]` conforme conclui.
>
> **Status:** Fases 0–5 ✅. Fase 6: 6.1 (guia) ✅ e 6.3 (docs) ✅; **falta só 6.2** (rodar no bag real do
> tanque e fixar tolerâncias) + 3.4 (extrator ROS 2) — ambos dependem dos seus dados/ambiente ROS 2.
> Verificação no container `kalibr` (D10); end-to-end com imagens reais na 6.2.

## Pré-requisitos

- Ambiente Docker do Kalibr (ROS 1 Noetic, `Dockerfile_ros1_20_04`) compilando com `catkin build`
  (ver `.ai/docs/stack.md`, `scripts/Makefile`).
- Calibração cam-IMU submersa existente (camchain-imucam.yaml + imu.yaml) disponível como insumo.
- Branch de feature criada: `calib_cam_imu_dvl` (D1).
- Decisões D1–D9 aprovadas.

## Fase 0 — Preparação

- [x] **0.1** Criar a branch `calib_cam_imu_dvl` a partir do estado atual do `kalibr`.
  - **Verificação:** `git -C kalibr rev-parse --abbrev-ref HEAD` retorna `calib_cam_imu_dvl`. ✓ (branch criada de `feature/calibration_zed`)
- [x] **0.2** Confirmar build limpo do pacote `kalibr` na branch antes de qualquer mudança.
  - **Verificação:** `catkin build kalibr` conclui sem erro; `TestErrorTerms` passa. ✓ (36 pkgs OK; 4/4 GTests
    passam) via `docker run -v <repo>:/catkin_ws/src/kalibr --entrypoint bash kalibr:latest` — fluxo de
    verificação padrão para as próximas tarefas.

## Fase 1 — Spike de viabilidade da expressão (de-risca D4)

- [x] **1.1 (spike)** Num script Python throwaway dentro do container, construir uma spline de pose de
  brinquedo e avaliar `poseSplineDv.linearVelocity(tk)`, `.orientation(tk).inverse()`,
  `.angularVelocityBodyFrame(tk)`, e compor `C_dvl_b*(C_b_w*v_w + w_b.cross(r_b))` com
  `RotationQuaternionDv`/`EuclideanPointDv`, passando a `ket.EuclideanError`.
  - **Verificação:** o script monta o `EuclideanError` e `evaluateError()` roda sem exceção; o valor
    numérico bate com o cálculo manual da velocidade do DVL para uma pose/lever-arm conhecidos.
    ✓ `||expr − manual|| = 0.0`, `evaluateError`≈0, whitening correto. Confirma **D4** (Python puro, sem C++
    no núcleo). Nota de ambiente: rodar Python standalone exige `LD_PRELOAD=libcholmod.so` (ver D10).
- [x] **1.2 (spike)** Verificar se `EuclideanExpression` pode ser multiplicada por um design variable
  escalar em Python (para a escala `s`). Se sim, registrar a API; se não, marcar o fallback C++ (D4).
  - **Verificação:** decisão registrada em `decisions.md` (D4/D7) — Python puro **ou** thin C++ `DvlVelocityError`.
    ✓ Resultado: `EuclideanExpression * ScalarExpression` **não** exposto no Python; `elementwiseMultiply`
    só com constante. → **geometria em Python puro; escala exige o thin C++ (ativa a tarefa 2.4).**

## Fase 2 — Modelo de medição e teste sintético (de-risca D9; TDD)

- [x] **2.1** Implementar um **gerador sintético**: dada uma trajetória (spline) conhecida, `T_dvl_imu` e
  escala `s` conhecidos, produzir amostras de velocidade do DVL (com ruído opcional) no formato de medição.
  - **Verificação:** para `T_dvl_imu`=I e `s`=1, a velocidade gerada iguala `C_b_w·v_w` da spline (tolerância numérica).
    ✓ `test/dvl/dvl_synthetic.py`. Trajetória analítica suave via `initPoseSplineSparse` (v/ω físicos:
    max|v|=0.45 m/s, max|ω|=1.25 rad/s); auto-check com 5 casos (física, identidade, escala, lever-arm, ruído).
- [x] **2.2 (teste primeiro)** Escrever um teste de **recuperação**: alimentar as amostras sintéticas no
  termo de erro do DVL + otimizador e checar que `T_dvl_imu`/`s` são recuperados dentro de tolerância.
  - **Verificação:** o teste falha sem a implementação e passa depois de 2.3 (rotação < ~0.1°, translação
    < ~mm, escala < 1e-3, com dados sem ruído). ✓ RED confirmado (`test/dvl/test_dvl_recovery.py`, exit 1:
    `DvlError` ausente). Cenário A (extrínseco, s=1) deve passar após 2.3; Cenário B (extrínseco+escala) após 2.4.
- [x] **2.3** Implementar o cálculo do resíduo de velocidade do DVL (Python puro, conforme spike 1.1),
  incluindo escala `s` (conforme 1.2) e whitening por covariância.
  - **Verificação:** o teste 2.2 passa. ✓ `python/kalibr_imu_camera_calibration/DvlError.py`
    (`addDvlVelocityErrorTerms`). Cenário A do teste: **rot_err=0.0000°, trans_err=0.00000 m** (recuperação
    exata do extrínseco). Escala fixa≠1 via `elementwiseMultiply`; escala estimada delega ao C++ (2.4).
- [ ] **2.4** (NECESSÁRIA — decidido em 1.2, para a escala `s`) Implementar
  `include/kalibr_errorterms/DvlVelocityError.hpp` + `src/DvlVelocityError.cpp`: error term `ErrorTermFs<3>`
  que recebe `(measurement, invR, predictedVelocity: EuclideanExpression, scale: ScalarExpression)` e computa
  resíduo `measurement − scale·predictedVelocity` (usa o `EuclideanExpression::operator*(ScalarExpression)`
  do C++). Adicionar ao `add_library(kalibr_errorterms ...)` no `CMakeLists.txt`, expor em `src/module.cpp`,
  e um caso em `test/TestErrorTerms.cpp`. A **geometria** (rotação/lever-arm) continua montada em Python (2.3).
  - **Verificação:** `catkin build kalibr` compila; `catkin run_tests kalibr` (GTest) passa o novo caso
    (resíduo e, se implementadas, jacobianas conferem numericamente). ✓ `DvlVelocityError` (deriva de
    `EuclideanError`, predição `predVel*scale`); GTest `testDvlVelocity` passa (harness de jacobiana, 5/5).
    Teste de recuperação Python **Cenário B verde**: `s_est=1.20000` (gt=1.2, erro 2e-16), extrínseco exato.
    (Warning benigno pré-existente `-Wdeprecated-copy` em `sm/Id.hpp` do Schweizer-Messer, não do nosso código.)

## Fase 3 — Configuração e ingestão de dados (R8, R12, D5)

- [x] **3.1** Definir e documentar o **schema do CSV do DVL** e criar um YAML de exemplo de config do DVL
  em `scripts/config/dvl0.yaml` (tópico/caminho CSV, sigma/cov default, `T_dvl_imu` inicial do xacro,
  `sound_speed`, escala inicial, limiares de gating).
  - **Schema CSV (16 col):** `timestamp_ns, vx, vy, vz, cov00..cov22 (9), velocity_valid, fom, altitude`.
  - **Verificação:** exemplos versionados; um parser lê o CSV de exemplo e imprime N amostras válidas.
    ✓ `scripts/config/dvl0.yaml` + `scripts/config/dvl0_example.csv` (5×16, 4 válidas/1 inválida). Schema
    documentado no cabeçalho do CSV.
- [x] **3.2** Implementar `DvlParameters(ParametersBase)` em `ConfigReader.py` (readYaml/writeYaml,
  getRosTopic/getCsvPath, defaults de ruído, extrínseco inicial, escala, `sound_speed`, gating),
  espelhando `ImuParameters` (`ConfigReader.py:428`).
  - **Verificação:** teste unitário de round-trip YAML (ler → objeto → escrever → reler) idempotente.
    ✓ `ConfigReader.py` (classe `DvlParameters`) + `test/dvl/test_dvl_config.py`: round-trip idempotente e
    construção via setters (`createYaml=True`) OK.
- [x] **3.3** Implementar `CsvDvlDatasetReader` (em `kalibr_common/`) iterando
  `(timestamp, velocity[3], invR[3x3], valid, fom, altitude)`, no estilo de `ImuDatasetReader.py`.
  Aplicar o gating de D7 aqui (ou marcar amostras inválidas).
  - **Verificação:** teste sobre um CSV sintético (da tarefa 2.1 exportado): nº de amostras e valores conferem;
    amostras inválidas são filtradas. ✓ `kalibr_common/DvlDatasetReader.py` + `write_csv` no gerador +
    `test/dvl/test_dvl_csv_reader.py`: 100 lidas, 96 válidas (4 gated), round-trip OK, integra com o termo (n_added=96).
- [x] **3.4** Reference extractor ROS 2: script que converte `dvl_msgs/DVL` de um bag ROS 2 no CSV do
  schema 3.1 (roda no lado do usuário; não faz parte do build ROS 1).
  - **Verificação:** documentado; executa sobre um bag de amostra e produz um CSV válido.
    ✓ `scripts/ros2_dvl_to_csv.py` (lib `rosbags`, lê MCAP/sqlite3 e `.zip` sem ROS 2; registra os tipos
    custom `dvl_msgs/DVL`/`DVLBeam`; `--list-topics`). Testado no bag real do tanque (`piscina_calib_01`):
    2116 amostras extraídas em ~6s, CSV 16-col válido (2061 válidas, covariância/fom reais).

## Fase 4 — Sensor `IccDvl` e integração no calibrador (R1–R5, R11, D6)

- [x] **4.1** Criar a classe `IccDvl` em `kalibr_imu_camera_calibration/IccSensors.py`, espelhando `IccImu`:
  `__init__` (carrega config + dados do DVL), `addDesignVariables(problem)` (`q_dvl_b_Dv`
  `RotationQuaternionDv`, `r_dvl_b_Dv` `EuclideanPointDv`, `scaleDv`, todos ativos), e
  `getResultTransformation()`/`updateDvlConfig()`.
  - **Verificação:** import e instanciação num teste; DVs aparecem no problema (contagem esperada).
    ✓ `IccDvl` + helpers `dvlExtrinsicToParams`/`dvlParamsToExtrinsic` (SE3↔C_dvl_b/r_b) + `test/dvl/test_iccdvl.py`.
    DVs criados, escala ativa/inativa conforme config, `getResultTransformation` reproduz o SE3 do config.
    Nota: `addDesignVariables(problem, group_id)` requer `inc.CalibrationOptimizationProblem` (pipeline real).
- [x] **4.2** Implementar `IccDvl.addVelocityErrorTerms(problem, poseSplineDv, ...)` usando o resíduo da
  Fase 2 (loop sobre as amostras, avaliar em `tk = stamp + timeOffset`, checar `t_min/t_max`, whitening,
  gating, `problem.addErrorTerm`). Espelha `IccImu.addAccelerometerErrorTerms` (`IccSensors.py:669`).
  - **Verificação:** num dataset sintético carregado via CSV, o nº de error terms adicionados/descartados
    é reportado e coerente. ✓ `IccDvl.addVelocityErrorTerms` (delega a `DvlError`, + `fallback_sigma`) +
    `test/dvl/test_iccdvl_recovery.py`: 400/400 termos, recuperação exata via IccDvl (rot 0°, trans 0, s=1.15).
- [x] **4.3** Implementar `IccDvl.findTimeOffsetPrior(referenceImu/poseSpline)` por correlação cruzada
  (D8), à moda de `IccImu.findOrientationPrior` (`IccSensors.py:782`).
  - **Verificação:** com um offset sintético injetado, o prior recuperado fica dentro de ~1 período de amostragem.
    ✓ `IccDvl.findTimeOffsetPrior` (correlação de `|v_dvl|` vs. velocidade da spline + refino, robustez de sinal)
    + `test/dvl/test_iccdvl_timeoffset.py`: offsets ±0.3/−0.2 s recuperados com erro 0.026 s (< 1 período 0.049 s); no-op quando desabilitado.
- [x] **4.4** Estender `IccCalibrator` (`IccCalibrator.py`): `DvlList`+`registerDvl`; incluir DVs do DVL em
  `initDesignVariables`; chamar `addVelocityErrorTerms` em `buildProblem`; `saveDvlParametersYaml`.
  Implementar Modo A/B (D6): flag que ativa/desativa os DVs cam-IMU e os inicializa da calibração fornecida.
  - **Verificação:** `buildProblem` roda de ponta a ponta num dataset sintético (cam+imu+dvl) sem exceção;
    no Modo A os DVs cam-IMU estão inativos (checagem de `isActive`).
    ✓ (parte sintética) `test/dvl/test_icccalibrator_dvl.py`: `registerDvl`, `saveDvlParametersYaml`
    (T_dvl_imu/escala/timeshift), e Modo A (`fixCamImuDesignVariables` desativa T_c_b/timeshift/q_i_b/r_b).
  - **DESVIO (verificação):** o `buildProblem` ponta a ponta exige imagens reais do AprilGrid (detecção de
    target no pipeline de câmera), não sintetizável offline. Deferido para o smoke test da CLI (Fase 5) e
    dados reais (Fase 6). Ver D11/D12.

## Fase 5 — CLI e saída (R7, R9, R10)

- [x] **5.1** Criar a CLI `python/kalibr_calibrate_dvl`, espelhando `kalibr_calibrate_imu_camera`
  (args `--bag`, `--cams`, `--imu`, `--target`, `--dvl` [config], `--dvl-csv`, `--recompute-cam-imu`,
  `--no-time-calibration`, `--max-iter`, `--recover-covariance`). Registra cam chain + imus + dvl,
  `buildProblem`, `optimize`, salva resultados. Adicionar em `catkin_install_python(PROGRAMS ...)`.
  - **Verificação:** `rosrun kalibr kalibr_calibrate_dvl --help` lista os args; roda num dataset sintético
    (bag cam+imu + CSV dvl) e termina gerando os arquivos. ✓ `--help` OK (imports/argparse limpos, grupo DVL
    com `--dvl/--dvl-csv/--recompute-cam-imu/--huber-dvl`); registrada no `CMakeLists.txt`. D12 implementado:
    `IccCalibrator.reuseProvidedCamImuExtrinsics` (lê `T_cam_imu` do `--cams` e inicializa cam0 antes de fixar).
  - **Nota:** o run ponta a ponta precisa de imagens reais do AprilGrid → verificado na Fase 6 (dados de tanque).
- [x] **5.2** Implementar a saída YAML estilo Kalibr do DVL: `T_dvl_imu` (4×4), `timeshift_dvl_imu` (s),
  `velocity_scale`, e um `*-results-dvl.txt` (RMS do resíduo, nº usados/descartados, escala, offset,
  covariância se `--recover-covariance`).
  - **Verificação:** os arquivos são gerados; os valores batem com o ground-truth sintético (dentro da tolerância de 2.2).
    ✓ `IccDvl.getResidualStats` + `IccCalibrator.saveDvlResultTxt` + wiring na CLI + `test/dvl/test_dvl_results.py`:
    RMS por-eixo ≈ σ (0.02), RMS norma ≈ √3·σ, contagens 400/400; txt com todos os campos.
    (`saveDvlParametersYaml` já implementado em 4.4.) **Covariância por-DVL do DVL: deferida** — o
    `--recover-covariance` recupera a do cam-IMU; extração específica dos DVs do DVL fica como refinamento.
- [x] **5.3** Adicionar um gráfico de resíduos de velocidade do DVL ao report (estilo `IccPlots.py`).
  - **Verificação:** o PDF/plot é gerado sem erro e mostra os resíduos por eixo. ✓ `IccPlots.plotDvlVelocityError`
    (residuos por eixo + bandas ±3σ) + integração em `IccUtil.generateReport` (loop `DvlList`) +
    `test/dvl/test_dvl_plot.py` (headless Agg → PDF 17 KB).

## Fase 6 — Validação em dados reais e documentação (CA5, CA7)

- [x] **6.1** Documentar o **procedimento de coleta em tanque** (bag combinado estéreo+IMU+DVL submerso,
  alvo visível + bottom-lock, excitação 6-DOF; extração do CSV do DVL) num doc no estilo `scripts/docs/kalibr.md`.
  - **Verificação:** doc revisável, com os comandos concretos ROS 2 → CSV → `kalibr_calibrate_dvl`.
    ✓ `scripts/docs/kalibr_dvl.md` (coleta, preparo, comandos, saídas, interpretação, Modo A/B, troubleshooting).
- [ ] **6.2** Rodar sobre o primeiro dataset real do tanque; fixar as tolerâncias numéricas de aceitação
  (RMS de velocidade, tolerância de translação vs. xacro) e o valor esperado de `velocity_scale` dado o
  `sound_speed` configurado.
  - **Verificação:** `velocity_scale` coerente com o sound-speed; RMS na faixa acordada; `T_dvl_imu`
    coerente com o xacro (CA5). Registrar os números finais na spec/decisions.
- [x] **6.3** Atualizar a documentação `.ai/docs/` do Kalibr (rodar `/ai_dev_kit:doc` em modo ATUALIZAR):
  novo doc de domínio da calibração de DVL, CLI em `api.md`, formatos em `data-model.md`.
  - **Verificação:** docs atualizados refletem `kalibr_calibrate_dvl`. ✓ `api.md` (CLI), `data-model.md`
    (dvl0.yaml/CSV/dvl-results), `domain/dvl-calibration.md` (novo), `domain/README.md`, `AGENTS.md`, `.state.json`.

## Riscos

- **Observabilidade fraca** (excitação insuficiente → braço de alavanca/escala mal determinados): mitigado
  por protocolo de captura 6-DOF (6.1) e recuperação de covariância (5.2) para flag de baixa observabilidade.
- **Bottom-lock intermitente no tanque** (superfície refletora fora de alcance): mitigado por gating (D7) e
  por validar viabilidade da captura antes (NEEDS CLARIFICATION da spec).
- **Sincronização de relógio ROS 2 → CSV** (deriva entre DVL e IMU): mitigado por `time_of_validity` como
  timestamp (R5) e pelo prior de offset (4.3); offset contínuo fica como evolução (D8).
- **Escala vs. sound-speed degenerada com escala da trajetória** se o alvo não fixar bem a métrica:
  mitigado pelo Modo A (cam-IMU/alvo fixos) e por checar `velocity_scale`≈esperado (6.2).
- **Suporte a escalar em Python** ausente: fallback thin C++ `DvlVelocityError` (2.4), já previsto.
