# Spec — Validação do fluxo de calibração no tanque (câmera-IMU-DVL)

> **O quê e por quê.** Fonte da verdade da intenção. Sem decisão técnica nem passos de implementação
> (esses vão em `decisions.md` e `plan.md`).

## Problema

Temos **4 rosbags de tanque** (`piscina_calib_01..04`), gravados submersos, cada um com **todos os
sensores** (estéreo ZED raw, IMU da ZED, IMU Microstrain, DVL A50, pressão) e atendendo aos requisitos
de captura para calibração câmera-IMU e câmera-IMU-DVL. É preciso **gerar a calibração atual** (há
pressa) e **validar o fluxo end-to-end** com dados reais: rodar câmera-IMU e DVL nos 4 bags e verificar
qualidade e consistência. A ferramenta `kalibr_calibrate_dvl` já existe e foi validada em dados
sintéticos (ver `.ai/specs/dvl-calibration/`); falta a validação em dados reais.

## Contexto

- **Bags:** `scripts/data/calibration/cam_imu_dvl/piscina_calib_0{1..4}` (MCAP em `.zip`, ~14 GB cada).
  Tópicos reais: `/zed/zed_node/left|right/color/raw/image` (raw color), `/imu/data` (Microstrain, ~197 Hz),
  `/zed/zed_node/imu/data` (ZED, ~98 Hz), `/dvl/data` (`dvl_msgs/DVL`, ~9 Hz), `/lar/bar/depth`, `/ekf/imu/data`.
- **Ferramentas prontas (Kalibr, este fork):** `kalibr_calibrate_imu_camera`, `kalibr_calibrate_dvl`
  (Modo A = cam-IMU fixo), o extrator `scripts/ros2_dvl_to_csv.py` (DVL→CSV, testado no bag 01: 2116 amostras).
- **Câmera:** o usuário tem os **intrínsecos + distorção** (e baseline estéreo). O Kalibr usa as imagens
  **raw + modelo de distorção** — **não é preciso retificar**.
- **IMU de referência:** **Microstrain** (`--imu` #1); ZED IMU é a segunda.
- **Ruído da IMU:** sem Allan variance por ora — usar valores de **datasheet/típicos** (provisório). A
  Allan variance é trabalho futuro (spec própria).
- **Referência de verdade:** o **xacro** serve de guia (extrínsecos nominais), sem ground-truth estrito;
  a validação principal é por **consistência entre os 4 bags** + qualidade dos resíduos.
- **DVL:** calibração online de feixes fora de escopo; extrínseco/escala/offset via `kalibr_calibrate_dvl`.

## Requisitos

- **R1 — Preparo dos dados (4 bags).** Extrair o CSV do DVL de cada bag (`ros2_dvl_to_csv.py`) e converter
  o bag ROS 2 → ROS 1 contendo os tópicos de **imagem estéreo + IMU** (Microstrain + ZED) necessários ao Kalibr.
- **R2 — Configs de entrada.** Montar, no formato Kalibr: `camchain.yaml` (intrínsecos + distorção + baseline,
  sobre os tópicos raw), `imu.yaml` (Microstrain + ZED, ruído provisório de datasheet), `target.yaml`
  (AprilGrid), `dvl0.yaml` (extrínseco inicial `T_dvl_imu` do xacro, `sound_speed`, tópico/CSV, gating).
- **R3 — Calibração câmera-IMU por bag.** Rodar `kalibr_calibrate_imu_camera` (Microstrain referência + ZED)
  em cada um dos 4 bags; obter `camchain-imucam.yaml` (`T_cam_imu`, timeshift) + `imu.yaml` por bag.
- **R4 — Calibração de DVL por bag.** Rodar `kalibr_calibrate_dvl` (Modo A, reusando o cam-IMU do mesmo bag)
  em cada um dos 4; obter `T_dvl_imu`, `velocity_scale`, `timeshift_dvl_imu`.
- **R5 — Comparação cross-bag (repetibilidade).** Comparar entre os 4 bags: `T_cam_imu`, `T_dvl_imu`
  (rotação e translação), `velocity_scale` e timeshifts. Reportar dispersão (média ± desvio).
- **R6 — Qualidade por bag.** Reportar erro de reprojeção (cam-IMU), RMS do resíduo de velocidade (DVL),
  nº de amostras de DVL usadas/descartadas (gating), e a `velocity_scale` vs. o esperado pelo `sound_speed`.
- **R7 — Aderência ao xacro (guia).** Comparar `T_cam_imu`/`T_dvl_imu` com os valores nominais do xacro,
  como sanidade (tolerância frouxa), não como critério estrito.
- **R8 — Consolidação.** Escolher/registrar a **calibração final** (o melhor bag ou uma média/mediana dos
  consistentes) e documentar os números e o procedimento.

## Não-objetivos

- **Allan variance / calibração de ruído da IMU.** Usar datasheet provisório; a estimação de ruído é
  **spec futura**.
- **Recalibrar intrínsecos da câmera.** Reusar os fornecidos (raw + distorção); nenhuma nova estimação de intrínsecos.
- **Retificar imagens.** O Kalibr usa raw + modelo de distorção; não gerar imagens retificadas.
- **Modo B (co-otimização cam-IMU-DVL).** Usar **Modo A** (cam-IMU fixo por bag).
- **Calibração online dos feixes do DVL (alpha/beta).** Fora de escopo.
- **Integração/validação no AQUA-SLAM.** Trabalho futuro (a conversão de formato e o teste downstream).
- **Alterar o código da ferramenta** salvo bug encontrado na validação (aí volta-se à spec `dvl-calibration`).

## Critérios de aceitação

- [ ] **CA1:** Os 4 bags têm dados preparados: `dvl0X.csv` extraído + bag ROS 1 (imagens+IMU) para o Kalibr.
- [ ] **CA2:** `kalibr_calibrate_imu_camera` roda nos 4 bags e conclui; o **erro de reprojeção** fica em faixa
      aceitável (a fixar com os dados — tipicamente < ~1 px).
- [ ] **CA3:** `kalibr_calibrate_dvl` roda nos 4 bags e conclui; produz `T_dvl_imu`/`velocity_scale`/timeshift;
      o **RMS do resíduo de velocidade** fica em faixa aceitável e o nº de amostras usadas/descartadas é reportado.
- [ ] **CA4 (consistência):** os extrínsecos (`T_cam_imu`, `T_dvl_imu`) e a `velocity_scale` são **consistentes
      entre os 4 bags** dentro de uma tolerância a fixar (ex.: rotação poucos graus, translação poucos cm, escala poucos %).
- [ ] **CA5:** a `velocity_scale` é coerente com o `sound_speed` configurado; `T_dvl_imu`/`T_cam_imu` são
      coerentes com o xacro dentro de uma tolerância frouxa (sanidade).
- [ ] **CA6:** há uma **calibração final** escolhida e um resumo (números por bag + consolidado) documentado.

## Perguntas em aberto

- **NEEDS CLARIFICATION (câmera):** os valores exatos de intrínsecos (`fx,fy,cx,cy`), coeficientes de
  distorção (e o modelo: `radtan`/`equidistant`), resolução e a **baseline estéreo** (ou `T_cn_cnm1`).
- **NEEDS CLARIFICATION (DVL):** o `T_dvl_imu` inicial do xacro e o `sound_speed` configurado no A50.
- **NEEDS CLARIFICATION (IMU):** valores de datasheet a usar para Microstrain e ZED IMU (noise density + random walk).
- **NEEDS CLARIFICATION (xacro):** os extrínsecos nominais (`T_cam_imu`, `T_dvl_imu`) para a comparação-guia.
- **NEEDS CLARIFICATION (tolerâncias):** faixas de aceitação (reprojeção, RMS DVL, consistência cross-bag)
  — a fixar após os primeiros resultados reais.
- **NEEDS CLARIFICATION (conversão):** confirmar que os tópicos de imagem+IMU convertem ROS 2 → ROS 1 sem
  os tipos custom (excluindo `/dvl/data` e `/lar/bar/depth`).
