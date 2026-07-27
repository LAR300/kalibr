---
description: Calibração de DVL no Kalibr (kalibr_calibrate_dvl) — modelo de medição, parâmetros estimados e arquitetura. Abrir ao mexer no termo de erro do DVL, no IccDvl ou na CLI de DVL.
sources: [aslam_offline_calibration/kalibr/src/DvlVelocityError.cpp, aslam_offline_calibration/kalibr/python/kalibr_imu_camera_calibration/DvlError.py, aslam_offline_calibration/kalibr/python/kalibr_imu_camera_calibration/IccSensors.py, aslam_offline_calibration/kalibr/python/kalibr_common/DvlDatasetReader.py, aslam_offline_calibration/kalibr/python/kalibr_calibrate_dvl, scripts/docs/kalibr_dvl.md]
---

# Calibração de DVL

> Extensão deste fork. Estima offline o extrínseco de um **DVL** (Water Linked A50) contra a IMU de
> referência, no framework batch/tempo-contínuo do Kalibr. Spec completa: `.ai/specs/dvl-calibration/`.

## O que estima
- **`T_dvl_imu`** (SE3, IMU→DVL): rotação + lever-arm.
- **`velocity_scale`**: escala de velocidade (erro de velocidade do som).
- **`timeshift_dvl_imu`**: offset temporal DVL↔IMU (prior por correlação; D8).

## Modelo de medição
O DVL não observa o alvo — só mede velocidade. A trajetória do corpo é a **B-spline de pose** do Kalibr
(reconstruída de câmera+IMU do próprio bag). O resíduo de cada amostra de DVL é:

```
v_dvl_pred = velocity_scale · C_dvl_b · ( C_b_w · linearVelocity(t) + ω_b × r_b )
resíduo    = v_dvl_pred − v_dvl_medido      (ponderado pela covariância do A50)
```
- **Geometria** montada em Python (`DvlError.py`, reusa `ket.EuclideanError`).
- **Escala** aplicada por um error term C++ (`DvlVelocityError`), pois `EuclideanExpression×escalar` não é
  exposto em Python (D4). GTest: `testDvlVelocity`.

## Arquitetura (onde vive cada peça)
- Termo de erro C++: `src/DvlVelocityError.cpp` (+ header, binding em `src/module.cpp`).
- Resíduo (Python): `python/kalibr_imu_camera_calibration/DvlError.py`.
- Sensor: `IccDvl` em `python/kalibr_imu_camera_calibration/IccSensors.py` (design variables `q_dvl_b`,
  `r_dvl_b`, `scale`; `addVelocityErrorTerms`; `findTimeOffsetPrior`; conversão SE3↔(C_dvl_b, r_b) — D11).
- Config: `DvlParameters` em `python/kalibr_common/ConfigReader.py`.
- Leitor de dados: `CsvDvlDatasetReader` em `python/kalibr_common/DvlDatasetReader.py` (gating D7).
- Orquestração: `IccCalibrator` (`DvlList`/`registerDvl`, `buildProblem` com `recompute_cam_imu`,
  `fixCamImuDesignVariables` = Modo A, `reuseProvidedCamImuExtrinsics` = D12, `saveDvlParametersYaml`,
  `saveDvlResultTxt`).
- CLI: `python/kalibr_calibrate_dvl`.
- Gráfico: `IccPlots.plotDvlVelocityError` (no relatório).

## Modo A (padrão) vs Modo B
- **Modo A:** reusa a calibração câmera-IMU submersa **fixa** (só trajetória/biases/DVL são livres). O
  `T_cam_imu` é inicializado do `--cams` fornecido (D12) antes de fixar.
- **Modo B (`--recompute-cam-imu`):** co-otimiza câmera-IMU-DVL num único batch.

## Ingestão de dados (ROS 2 → ROS 1)
O A50 é ROS 2 (`dvl_msgs/DVL`); o Kalibr é ROS 1. O stream do DVL entra por **CSV** (16 colunas, schema em
`scripts/config/dvl0_example.csv`), evitando regerar a mensagem custom em ROS 1 (D5).

## Fora de escopo (não-objetivos)
Calibração da orientação dos feixes (alpha/beta), recalibrar câmera/IMU por padrão, emitir formato
AQUA-SLAM, fusão online. Ver `.ai/specs/dvl-calibration/spec.md`.

## Procedimento de uso
Guia completo (coleta em tanque, comandos, interpretação): `scripts/docs/kalibr_dvl.md`.
