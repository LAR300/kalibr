---
description: Índice das regras de domínio do Kalibr (uma por arquivo). Abrir ao mexer no fluxo de calibração ou na integração com sistemas VIO/SLAM.
sources: [scripts/docs/kalibr.md, aslam_offline_calibration/kalibr/python/]
---

# Domínio — regras de negócio

> Regras detalhadas do fluxo de calibração, uma por arquivo.

- `cam-imu-calibration.md` — o fluxo completo câmera+IMU (o caso de uso do fork, ZED 2i) e a **ponte
  de integração** para consumir as saídas do Kalibr em sistemas VIO/SLAM (ex: AQUA-SLAM).
- `dvl-calibration.md` — a calibração de **DVL** (`kalibr_calibrate_dvl`): modelo de medição, parâmetros
  estimados (`T_dvl_imu`, escala, offset) e arquitetura. Extensão deste fork.

Guia detalhado de operação (passo a passo, troubleshooting): `scripts/docs/kalibr.md` na raiz do repo.
