---
description: Decisões estruturais do Kalibr (workspace catkin multi-pacote, otimização batch com B-splines) e o porquê. Abrir ao mexer em pacotes, no pipeline de otimização, ou ao reaproveitar módulos numa integração.
sources: [Schweizer-Messer/, aslam_optimizer/, aslam_cv/, aslam_nonparametric_estimation/, aslam_incremental_calibration/, aslam_offline_calibration/kalibr/python/kalibr_imu_camera_calibration/]
---

# Arquitetura

> Decisões estruturais e o **porquê** delas. O que reaproveitar numa integração vive aqui.

## Visão geral

Kalibr é um **workspace catkin com ~38 pacotes** organizados em camadas. Núcleo em C++ com bindings
Python; a lógica de calibração de alto nível é Python, chamando o otimizador C++ via bindings.

```
  camada de utilitários     Schweizer-Messer  (sm_common, sm_eigen, sm_kinematics, sm_boost,
                             numpy_eigen, sm_property_tree, ...)
        │
  camada de base            aslam_optimizer            aslam_cv              aslam_nonparametric_estimation
                            (aslam_backend,            (aslam_cameras,        (bsplines,
                             aslam_backend_expressions, aslam_cameras_april,   aslam_splines)
                             sparse_block_matrix)       aslam_cv_error_terms)
        │
  camada de calibração      aslam_incremental_calibration  (incremental_calibration)
        │
  aplicação                 aslam_offline_calibration/kalibr  ← pacote principal (CLIs + lógica Python
                                                                + error terms de IMU em C++)
                            aslam_offline_calibration/ethz_apriltag2  (detecção AprilTag)

  infra de build            catkin_simple, opencv2_catkin
```

## Decisões de arquitetura (ADRs leves)

### Estimação batch em tempo contínuo com B-splines
- **Contexto:** câmera e IMU amostram em taxas diferentes e assíncronas; alinhar por interpolação discreta perde precisão.
- **Decisão:** modelar a **trajetória do sensor como uma B-spline de pose contínua** (`bsplines.BSplinePose`, ordem 6, `~100` knots/s) e os **biases da IMU como B-splines euclidianas**. Otimizar tudo num único problema batch.
- **Por quê:** permite avaliar pose/velocidade/aceleração em qualquer instante (inclusive o timeshift câmera-IMU como variável contínua), fundindo medidas assíncronas sem reamostrar.
- **Consequências:** o motor precisa de design variables de spline e de um solver de mínimos quadrados esparso próprios (por isso `aslam_splines` + `aslam_backend`).

### Otimizador próprio (aslam_backend), não Ceres/g2o
- **Contexto:** precisavam de um backend batch com grafo de expressões diferenciáveis e álgebra esparsa por blocos.
- **Decisão:** `aslam_backend` + `aslam_backend_expressions` + `sparse_block_matrix` (este último herda de código estilo g2o), com recuperação de covariância.
- **Consequências:** `IccCalibrator` usa `aopt.Optimizer2`; a estrutura do problema vem de `incremental_calibration` (`CalibrationOptimizationProblem`), que também analisa **observabilidade** (QR / informação mútua).

### Lógica em Python, núcleo pesado em C++ via bindings
- **Contexto:** iterar rápido na orquestração da calibração, sem abrir mão de desempenho no otimizador.
- **Decisão:** CLIs e pipeline em Python (`kalibr_*_calibration/`), chamando C++ via `numpy_eigen`/boost-python; error terms de IMU implementados em C++ (`src/{Accelerometer,Gyroscope,Euclidean}Error.cpp`) expostos como `kalibr_errorterms`.
- **Consequências:** ao portar/integrar, o ponto de entrada natural é o Python (`ConfigReader`, `IccSensors`, `IccCalibrator`), não o C++.

### Migração para Python 3
- **Contexto:** upstream migrou (PR #515, 2022).
- **Decisão/consequência:** o código Python é Python 3. O `Dockerfile_ros1_16_04` (Kinetic) ainda usa Python 2.7; prefira **20.04/Noetic** (usado pelo `scripts/Makefile`).

## Pipeline CAM-IMU (onde tudo acontece)

Em `aslam_offline_calibration/kalibr/python/kalibr_imu_camera_calibration/`:

- **`IccSensors.py`** — `IccCamera`, `IccCameraChain`, `IccImu`:
  - Trajetória: `BSplinePose(order=6, RotationVector())`, inicializada das poses recuperadas da câmera (`initPoseSplineFromCamera`).
  - Gravidade estimada no frame mundo (`gravity_w ≈ 9.80655`) a partir da força específica média.
  - IMU: B-splines de bias (gyro/accel). Termos de erro: `addCameraErrorTerms` (reprojeção), `addAccelerometerErrorTerms`, `addGyroscopeErrorTerms`.
- **`IccCalibrator.py`** — monta e resolve:
  - `initDesignVariables`: `BSplinePoseDesignVariable`, `gravityDv`, design vars de IMU e da cadeia de câmeras.
  - `buildProblem`: `inc.CalibrationOptimizationProblem`.
  - `optimize`: `aopt.Optimizer2` (+ recuperação de covariância opcional).

## Módulos reaproveitáveis numa integração

Se a integração precisar de calibração/geometria dentro de outro sistema, os candidatos são:
- `bsplines` / `aslam_splines` — trajetória contínua.
- `aslam_backend` (+ expressions) — otimizador batch esparso.
- `aslam_cv_error_terms` + `kalibr_errorterms` — termos de reprojeção / IMU.
- `aslam_cameras` (+ `aslam_cameras_april`) — modelos de câmera e detecção de AprilGrid.
- `python/kalibr_common/ConfigReader.py` — parse/validação dos YAMLs (câmera, IMU, target).

Na prática, porém, a integração mais provável **não é** reaproveitar código: é usar as **saídas** do
Kalibr (YAMLs de `T_cam_imu`, intrínsecos, ruído IMU) como **entrada** de outro sistema. Ver
`domain/cam-imu-calibration.md`.

## Cadeia de dependências (resumida)
`Schweizer-Messer` → `aslam_optimizer` / `aslam_cv` / `bsplines` → `aslam_splines` + `incremental_calibration` → `aslam_offline_calibration/kalibr`.

## O que está "pela metade" / fora de foco neste fork
- O upstream suporta **IMU-IMU** e **rolling shutter**, mas o `scripts/` deste fork só documenta e configura **CAM-IMU** (ZED 2i). Não presuma que o fluxo IMU-IMU/RS está exercitado aqui.
- Diretório `.ai/` (fora de `.ai/docs/`) existia vazio no checkout — propósito indeterminado.
