# Decisões — Calibração de DVL no Kalibr

> ADR leve. Registra o **porquê** técnico. Durável.

### D1 — Desenvolver dentro do pacote `kalibr`, em branch de feature

- **Contexto:** o objetivo do usuário é que o Kalibr concentre toda a calibração; o repo já é um fork
  (o commit atual adicionou `scripts/`).
- **Decisão:** implementar no próprio pacote `aslam_offline_calibration/kalibr`, na branch
  `calib_cam_imu_dvl`, seguindo a organização existente (error terms em `src/`+`include/kalibr_errorterms/`,
  sensores em `python/kalibr_imu_camera_calibration/`, CLIs em `python/`).
- **Por quê:** reuso direto da maquinaria (spline, backend, ConfigReader, IccCalibrator) sem glue
  entre pacotes; unidade coesa. Alternativa descartada: pacote catkin separado (mais boilerplate,
  precisaria re-expor internals; o valor de "upstream limpo" não se aplica a um fork).
- **Consequências:** diverge do upstream `ethz-asl/kalibr` (merge futuro mais difícil) — aceito.

### D2 — Frame de referência: a IMU de referência (Microstrain) → `T_dvl_imu`

- **Contexto:** Kalibr é IMU-cêntrico (spline no frame da IMU de referência); AQUA-SLAM é câmera-cêntrico.
  A Microstrain foi a `--imu` #1 na calibração existente.
- **Decisão:** estimar o extrínseco do DVL no frame da IMU de referência: `T_dvl_imu`.
- **Por quê:** o braço de alavanca do DVL só é observável via `v_dvl = v_imu + ω×r` (giroscópio); manter
  no frame da IMU deixa a física direta e evita compor por `T_cam_imu`. Alternativa (câmera, como o
  AQUA-SLAM) adiciona composição e propagação de erro.
- **Consequências:** para obter `T_dvl_c` (formato AQUA-SLAM, futuro) compõe-se com `T_cam_imu` já conhecido.

### D3 — Observação por velocidade 3D; alpha/beta fora de escopo

- **Decisão:** usar o vetor `velocity` (vx/vy/vz) do A50 como observação; não calibrar orientação dos feixes.
- **Por quê:** os parâmetros pedidos (extrínseco, tempo, escala) são todos observáveis pela velocidade 3D;
  alpha/beta têm observabilidade fraca e a geometria do A50 é conhecida. Simplifica o termo de erro.
- **Consequências:** não precisamos das velocidades por-feixe para o núcleo (só para gating). Extensão
  futura a alpha/beta permanece possível (portar o modelo de feixes do AQUA-SLAM), mas não implementada.

### D4 — Resíduo montado em Python reusando `EuclideanError` (sem C++ novo para o núcleo)

- **Contexto:** os termos de acelerômetro/giroscópio (`IccImu`) constroem a predição como uma
  `EuclideanExpression` a partir da spline + design variables e passam a `ket.EuclideanError`
  (`IccSensors.py:698-703, 734-741`). Verificado: `BSplinePoseDesignVariable.linearVelocity(tk)` retorna
  `EuclideanExpression` (exposto em `aslam_splines_python/spline_module.cpp:63`); `RotationExpression *
  EuclideanExpression` e `EuclideanExpression.cross(...)` existem.
- **Decisão:** montar a velocidade predita do DVL puramente em Python:
  `v_dvl_pred = C_dvl_b * ( C_b_w * poseSplineDv.linearVelocity(tk) + ω_b.cross(r_b) )` e usar
  `ket.EuclideanError(v_meas, invR·peso, v_dvl_pred)`. **Sem novo error term C++ para o núcleo.**
- **Por quê:** menor risco e menos superfície; espelha o padrão validado do acelerômetro.
- **Consequências:** só a **escala** `s` pode exigir suporte a escalar (ver D7). Se a multiplicação
  `EuclideanExpression`×escalar-DV não estiver exposta em Python, cria-se um **thin C++ `DvlVelocityError`**
  que recebe a expressão predita + `ScalarDesignVariable` de escala (fallback, decidir após spike T2).
- **RESOLVIDO (spike 1.2):** geometria em Python puro **confirmada** (spike 1.1: `||expr−manual||=0`).
  A multiplicação `EuclideanExpression×ScalarExpression` **NÃO está exposta** no Python (só `elementwiseMultiply`
  com constante). Portanto o **thin C++ `DvlVelocityError` é necessário** para a escala (tarefa 2.4 do plano),
  ficando isolado no pacote `kalibr` (o `operator*(ScalarExpression)` existe no C++). Não mexer em `aslam_backend_python`.

### D5 — Ingestão do DVL por CSV (schema fixo), com caminho ROS 1 opcional

- **Contexto:** bags gravados em ROS 2; Kalibr é ROS 1; `dvl_msgs/DVL` é mensagem custom.
- **Decisão:** o leitor de dataset do DVL consome um **CSV** (schema em `plan.md` T4), extraído no lado
  ROS 2. Formato no espírito do `imu0.csv` do Kalibr.
- **Por quê:** evita regerar/registrar a mensagem custom em ROS 1 para desserializar bag convertido —
  caminho de menor atrito e testável offline. Alternativa (regerar `dvl_msgs` em ROS 1) fica como
  suporte opcional a leitura por tópico.
- **Consequências:** precisamos definir e documentar o schema do CSV e um extrator de referência (ROS 2).

### D6 — Modo A (cam-IMU fixo) por padrão; Modo B (conjunto) por flag

- **Contexto:** a spline é reconstruída do próprio bag do DVL; os **parâmetros** cam-IMU já foram
  calibrados submersos.
- **Decisão:** por padrão, inicializar `T_cam_imu`/timeshift/intrínsecos a partir da calibração fornecida
  e **desativar** (`setActive(False)`) esses design variables — só spline (trajetória), biases e DVL são
  otimizados. Flag `--recompute-cam-imu` reativa os DVs cam-IMU (Modo B, conjunto).
- **Por quê:** robustez e respeito à calibração já validada; isola a estimação do DVL. Modo B disponível
  para quem quiser co-otimização máxima.
- **Consequências:** precisa de um caminho para carregar e travar o resultado cam-IMU prévio.

### D7 — Escala como design variable escalar; ponderação e gating pelos campos do A50

- **Decisão:** estimar a escala `s` como um design variable escalar (init 1.0). Ponderar cada resíduo
  (whitening) pela `covariance` do A50 (fallback: `fom` ou um sigma default). Descartar amostras com
  `velocity_valid=false`, `fom` acima de limiar, `altitude` fora de faixa, ou feixes inválidos.
- **Por quê:** `s` captura erro de sound-speed e é observável porque a escala da trajetória é fixada pelo
  alvo; usar a covariância nativa dá pesos estatisticamente corretos. Gating remove medições sem bottom-lock.
- **Consequências:** a observabilidade de `s` depende de haver escala métrica confiável (alvo) — reforça o Modo A.

### D8 — Offset temporal: prior por correlação cruzada; DV contínuo como evolução

- **Decisão:** inicializar o offset DVL↔IMU por correlação cruzada entre `|v_dvl|` e a velocidade da
  spline (análogo ao prior IMU-IMU em `IccSensors.findOrientationPrior`), avaliando a spline em `tk+offset`.
  Estimação contínua do offset (DV, à la timeshift de câmera) fica como melhoria posterior.
- **Por quê:** entrega valor cedo com baixo risco; o offset contínuo exige mais encanamento.
- **Consequências:** primeira versão pode tratar o offset como prior fixo refinável; documentar limitação.

### D10 — Ambiente de verificação: container `kalibr` + `LD_PRELOAD=libcholmod.so`

- **Contexto:** o host não tem ROS/aslam; a imagem docker `kalibr:latest` tem o workspace buildado.
  Descoberto no spike 1.1: importar os módulos Python do Kalibr standalone falha com
  `libbsplines.so: undefined symbol: cholmod_solve`.
- **Decisão:** verificar tudo dentro do container via
  `docker run --rm -v <repo>:/catkin_ws/src/kalibr --entrypoint bash kalibr:latest -lc '...'`,
  fazendo `source /catkin_ws/devel/setup.bash`; para rodar scripts/testes **Python**, prefixar
  `LD_PRELOAD=/lib/x86_64-linux-gnu/libcholmod.so`.
- **Por quê:** o C++ (gtest) linka SuiteSparse corretamente, mas o carregamento Python não resolve
  `cholmod` sozinho; o preload é o fix robusto. (Alternativa: ajustar ordem de import — menos confiável.)
- **Consequências:** os testes Python das Fases 2–5 devem documentar/usar esse preload; considerar
  encapsular num pequeno wrapper de teste.

### D11 — Convenção de `T_dvl_imu`: SE3 na fronteira, (C_dvl_b, r_b) internamente

- **Contexto:** o resíduo usa a rotação `C_dvl_b` (IMU→DVL) e o lever-arm `r_b` (posição do DVL no frame
  IMU). Usuário/Kalibr esperam uma transformação padrão (como `T_cam_imu`).
- **Decisão:** design variables internos são `q_dvl_b` (C_dvl_b) e `r_dvl_b` (r_b). O YAML de config e de
  saída usa `T_dvl_imu` como **SE3 padrão** (IMU→DVL): `p_dvl = C_dvl_b·p_imu + t`, com `t = -C_dvl_b·r_b`.
  Conversão em `IccSensors.dvlExtrinsicToParams`/`dvlParamsToExtrinsic`.
- **Por quê:** entrega ao usuário um SE3 convencional (coerente com `T_cam_imu`) sem perder a
  parametrização física natural (rotação + lever-arm) usada pela otimização.
- **Consequências:** o gerador sintético (test) usa o packing interno (C, r_b) direto; os testes de
  recuperação comparam parâmetros internos; a conversão SE3 tem teste de round-trip próprio.

### D12 — Modo A: inicializar o cam-IMU fixado a partir da calibração fornecida (pendência da CLI)

- **Contexto:** `fixCamImuDesignVariables` desativa os DVs cam-IMU **no valor em que estão**. No fluxo
  atual do Kalibr, `cam0.T_extrinsic` é inicializado por `findOrientationPriorCameraChainToImu` (estimativa
  a partir dos dados), não necessariamente pelo `T_cam_imu` do `--cams` fornecido.
- **Decisão:** a CLI `kalibr_calibrate_dvl` (Fase 5) deve **inicializar** `cam0.T_extrinsic` (e os
  timeshifts/extrínsecos imu-imu) a partir da calibração cam-IMU fornecida **antes** de fixar (Modo A),
  para que o valor fixado seja o da calibração submersa já validada — não uma re-estimativa.
- **Por quê:** Modo A significa "reusar a calibração existente"; fixar numa re-estimativa contrariaria isso.
- **Consequências:** item a resolver no wiring da CLL (tarefa 5.1); marcado como `NEEDS CLARIFICATION`
  de implementação. Em Modo B (joint) não se aplica (DVs ficam ativos).

### D9 — De-riscar com dados sintéticos antes do tanque real

- **Decisão:** validar o termo de erro e a recuperação dos parâmetros num **dataset sintético** (spline
  conhecida → velocidades geradas com `T_dvl_imu`/escala conhecidos) antes de usar o bag real.
- **Por quê:** separa erro de implementação de problema de dados/observabilidade; dá um teste de aceitação
  determinístico (CA6) independente de coletar dados no tanque.
- **Consequências:** uma das primeiras tarefas do plano é o gerador sintético + teste de recuperação.
