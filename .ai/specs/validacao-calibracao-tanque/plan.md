# Plano — Validação do fluxo de calibração no tanque

> Micro-tarefas verificáveis. O "porquê" das escolhas está em `decisions.md` (D1–D6); os requisitos em
> `spec.md` (R1–R8). Marque `[x]` conforme conclui.
>
> **Legenda:** 🔴 = bloqueado por insumo do usuário · 🐳 = roda no container Kalibr · 💻 = roda no host.
> **Status:** não iniciado.

## Fase 0 — Ambiente e insumos

- [ ] **0.1 🐳 Container com o código do DVL.** `make clean && make run` (na pasta `scripts/`), depois
  `catkin build kalibr && source devel/setup.bash` dentro do container.
  - **Verificação:** `rosrun kalibr kalibr_calibrate_dvl --help` funciona; `rosrun kalibr kalibr_calibrate_imu_camera --help` também.
- [ ] **0.2 🔴 Coletar os insumos do usuário** e registrá-los:
  - Câmera: `fx,fy,cx,cy`, `distortion_model` + `distortion_coeffs` (esq. e dir.), `resolution`, baseline (`T_cn_cnm1`).
  - DVL: `T_dvl_imu` inicial (xacro) e `sound_speed` do A50.
  - IMU: valores de datasheet (noise density + random walk) para Microstrain e ZED; `update_rate` de cada.
  - Xacro: extrínsecos nominais (`T_cam_imu`, `T_dvl_imu`) para a comparação-guia.
  - **Verificação:** valores anotados (ex.: num `scratch.md` ou nos próprios yamls da Fase 2).

## Fase 1 — Preparo dos dados (4 bags)

- [ ] **1.1 💻 Extrair o CSV do DVL dos 4 bags** com `ros2_dvl_to_csv.py` (bag 01 já feito).
  - **Verificação:** 4 arquivos `dvl0X.csv`; contagem de amostras coerente (~2 k cada) e válidas reportadas.
- [ ] **1.2 💻 Converter ROS 2 → ROS 1** (imagens estéreo + IMUs) dos 4 bags, excluindo `/dvl/data` e
  `/lar/bar/depth` (D1). Testar primeiro no bag 01.
  - **Verificação:** `rosbag info` do bag ROS 1 lista os tópicos de imagem + `/imu/data` + `/zed/.../imu/data`
    com contagens coerentes; sem erro de tipo custom.

## Fase 2 — Configs de entrada (Kalibr)

- [x] **2.1 Montar `camchain.yaml`** (2 câmeras, tópicos raw, intrínsecos + distorção + `T_cn_cnm1`) — D2.
  - **Verificação:** parse OK; `rostopic` batem com os tópicos do bag ROS 1; resolução confere.
    ✓ `config01/camchain.yaml` (radtan 4-params, k3 dropado; baseline 0.1211 m; `T_cn_cnm1` da ZED).
    Resolução confirmada no bag: **1280×720** (bate). ⚠️ encoding **bgra8** — checar no passo 3.1.
- [x] **2.2 Montar `imu.yaml`** (Microstrain #1 + ZED) com ruído de datasheet (D3); `rostopic`/`update_rate` corretos.
  - **Verificação:** parse OK; tópicos batem com o bag.
    ✓ `config01/imu.yaml` (Microstrain 3DM-GV7, `/imu/data`, rate 200, ruído provisório). 1º run com
    **Microstrain-só** (ZED IMU adicionada depois — D5). Valores de ruído a reavaliar (Allan variance).
- [x] **2.3 `target.yaml` + `dvl0.yaml` por bag** (`T_dvl_imu` inicial do xacro, `sound_speed`, caminho do CSV,
  gating). `target.yaml` = o AprilGrid usado.
  - **Verificação:** parse OK (via `DvlParameters`); `dvl0.yaml` aponta para o `dvl0X.csv` certo.
    ✓ `config01/target.yaml` (AprilGrid 6×6, tagSize 0.088 — **confirmar** que bate com o alvo físico) +
    `config01/dvl0.yaml` (csv `/data/.../dvl0_calib01.csv`, sound_speed 1500, T_dvl_imu identidade).

## Fase 3 — Calibração câmera-IMU (por bag)

- [x] **3.1 🐳 Rodar `kalibr_calibrate_imu_camera` no bag 01** (Microstrain ref + ZED; D5). Depurar aqui.
  - **Verificação:** conclui; **erro de reprojeção** em faixa aceitável (CA2); `T_cam_imu` plausível vs. xacro;
    gera `*-camchain-imucam.yaml` + `*-imu.yaml`.
    ✓ Convergiu (Microstrain-só). Reprojeção 1.53/1.59 px (mediana ~1.24/1.37 — **elevada pelo k3 dropado**,
    D7). Accel 0.036 m/s², giro 0.0063 rad/s, **gravidade 9.807** ✓. `T_cam0_imu0` trans `[0.128,0.021,-0.135]`
    (≈0.187 m vs. xacro ~0.227 m — ordem certa). timeshift 16.8 ms. Saídas em `data/output/piscina_calib_01-*`.
  - **Ajustes de prep necessários (registrados):** mono8 na conversão; **rebasing de timestamps (D8)**;
    **`--timeoffset-padding 0.1`** (timeshift real ~56 ms > 30 ms padrão).
- [ ] **3.2 🐳 Rodar nos bags 02, 03, 04.**
  - **Verificação:** os 4 concluem; reprojeção registrada por bag.

## Fase 4 — Calibração de DVL (por bag)

- [ ] **4.1 🐳 Rodar `kalibr_calibrate_dvl` no bag 01** (Modo A, reusando o cam-IMU do bag 01 + `dvl01.csv`).
  - **Verificação:** conclui; gera `T_dvl_imu`/`velocity_scale`/timeshift; **RMS** e nº usados/descartados
    reportados (CA3); ajustar gating do `dvl0.yaml` se necessário (amostras de baixa altitude/velocidade alta).
- [ ] **4.2 🐳 Rodar nos bags 02, 03, 04.**
  - **Verificação:** os 4 concluem; resultados salvos por bag.

## Fase 5 — Comparação e consolidação

- [ ] **5.1 💻 Script de comparação cross-bag** (R5): lê os 4 resultados e reporta `T_cam_imu`, `T_dvl_imu`
  (ângulo de rotação + translação), `velocity_scale` e timeshifts — média ± desvio entre os bags.
  - **Verificação:** tabela impressa; dispersão calculada.
- [ ] **5.2 Avaliar consistência + xacro + escala** (CA4/CA5): fixar as tolerâncias com base nos números;
  comparar com o xacro (guia) e a escala vs. `sound_speed`.
  - **Verificação:** dispersão dentro de faixa razoável (a definir); desvios grandes → investigar (excitação/gating/sinc).
- [ ] **5.3 Escolher a calibração final + documentar** (R8/CA6): melhor bag ou mediana dos consistentes;
  resumo por bag + consolidado (ex.: em `scripts/data/output/` e um doc curto).
  - **Verificação:** calibração final registrada; procedimento e números documentados.

## Riscos

- **Conversão ROS 2 → ROS 1 pesada** (imagens raw color, ~GB): tempo/disco; mitigar convertendo por bag e
  só os tópicos necessários (D1); validar no bag 01 antes dos demais.
- **Ruído de IMU provisório (datasheet):** pode elevar resíduos do cam-IMU; aceitável, registrar; melhora com Allan (futuro).
- **Excitação/observabilidade insuficiente em algum bag:** lever-arm/escala mal determinados; mitigado pela
  comparação cross-bag (o bag ruim se destaca) e pelo gating.
- **Distortion model errado** (radtan vs equidistant): reprojeção alta; conferir com o insumo 0.2 e o report.
- **Mismatch de tópicos** entre yaml e bag: `Could not find topic`; conferir na Fase 2.
