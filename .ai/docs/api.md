---
description: Superfície de uso do Kalibr — as ferramentas de linha de comando (rosrun kalibr *), suas entradas e saídas. Abrir ao rodar uma calibração ou automatizar o fluxo. (Kalibr não expõe API web; a interface são CLIs + tópicos ROS.)
sources: [aslam_offline_calibration/kalibr/python/kalibr_calibrate_cameras, aslam_offline_calibration/kalibr/python/kalibr_calibrate_imu_camera, aslam_offline_calibration/kalibr/python/kalibr_bagcreater, aslam_offline_calibration/kalibr/python/exporters/]
---

# Interface (CLIs)

> Kalibr **não tem API web**. A "interface" são executáveis `rosrun kalibr <tool>` que leem YAML/rosbag
> e escrevem YAML/PDF, mais os tópicos ROS (`sensor_msgs/Image`, `sensor_msgs/Imu`) dentro dos bags.
> Todas ficam em `aslam_offline_calibration/kalibr/python/`.

## Convenções gerais
- Invocação: `rosrun kalibr <ferramenta> [flags]` (dentro do container ROS).
- Requer `roscore` acessível (rode-o no host ou no container).
- Entradas por flags (`--bag`, `--target`, ...); saídas gravadas no diretório corrente.

## Ferramentas principais

### `kalibr_bagcreater`
- **Propósito:** montar um ROS bag a partir de dados brutos em pastas.
- **Entrada:** `--folder <dir>` com subpastas `cam*/` (imagens nomeadas por timestamp em ns, `.png/.jpg/.bmp`) e `imu*.csv` (7 colunas: `timestamp[ns], omega_xyz[rad/s], alpha_xyz[m/s²]`).
- **Saída:** `--output-bag <arquivo.bag>`. Tópicos: `cam0/` → `/cam0/image_raw`; `imu0.csv` → `/imu0`.

### `kalibr_calibrate_cameras`
- **Propósito:** intrínsecos e extrínsecos de um sistema de câmeras (mesmo sem FOV sobreposto global).
- **Entrada:** `--bag`, `--topics <t0 t1 ...>`, `--models <par câmera-distorção>` (ex.: `pinhole-radtan`, `pinhole-equi`, `omni-radtan`, `eucm-none`, `ds-none`), `--target <target.yaml>`. Opcional `--dont-show-report`, `--export-poses`.
- **Saída:** `camchain-*.yaml` (intrínsecos + `T_cn_cnm1`), report PDF, opcional CSV de poses `[time_ns, pos, quat]`.

### `kalibr_calibrate_imu_camera`
- **Propósito:** calibração espacial + temporal IMU↔câmeras (o fluxo central para VIO/SLAM).
- **Entrada:** `--bag`, `--cams <camchain.yaml>`, `--imu <imu0.yaml [imu1.yaml ...]>` (primeira é referência), `--target <target.yaml>`. Opcionais: `--imu-models {calibrated, scale-misalignment, scale-misalignment-size-effect}`, `--no-time-calibration`, `--max-iter 30`, `--recover-covariance`, `--timeoffset-padding`, `--show-extraction`, `--bag-freq <Hz>`.
- **Saída:** `*-camchain-imucam.yaml` (`T_cam_imu` 4×4 + `timeshift_cam_imu`), `*-imu.yaml`, `*-results-imucam.txt`, `*-report-imucam.pdf`.

### `kalibr_calibrate_dvl` (extensão deste fork)
- **Propósito:** calibração offline de um **DVL** (Water Linked A50) contra a IMU de referência —
  estima `T_dvl_imu` (SE3), `velocity_scale` (sound-speed) e `timeshift_dvl_imu`, reusando a calibração
  câmera-IMU submersa existente. Ver `domain/dvl-calibration.md` e `scripts/docs/kalibr_dvl.md`.
- **Entrada:** `--bag` (câmera+IMU, para reconstruir a spline), `--cams <camchain-imucam.yaml>` (com `T_cam_imu`),
  `--imu <imu.yaml>`, `--target`, `--dvl <dvl0.yaml>`, `--dvl-csv <dvl0.csv>` (stream do DVL, 16 colunas).
  Opcionais: `--recompute-cam-imu` (Modo B, co-otimiza cam-IMU-DVL; padrão é Modo A com cam-IMU fixo),
  `--huber-dvl <w>`, `--no-time-calibration`, `--max-iter`, `--recover-covariance`.
- **Saída:** `*-dvl-results.yaml` (`T_dvl_imu`, `velocity_scale`, `timeshift_dvl_imu`), `*-results-dvl.txt`
  (estatísticas de resíduo), `*-report-dvl.pdf`, e o `*-camchain-imucam.yaml`/`*-imu.yaml` reusados.

### `kalibr_calibrate_rs_cameras`
- **Propósito:** intrínsecos completos de uma câmera **rolling shutter** (projeção, distorção, shutter). Fora do foco do fork.

## Utilitários
- `kalibr_bagextractor` — extrai imagens/IMU de um bag (`--image-topics`, `--imu-topics`).
- `kalibr_create_target_pdf` — gera PDF do padrão de calibração.
- `kalibr_camera_validator` — valida intrínsecos ao vivo. `kalibr_camera_focus` — ajuda no foco.
- `kalibr_visualize_calibration`, `kalibr_visualize_distortion` — visualizações.

## Exportadores (`python/exporters/`)
Convertem os resultados para outros frameworks VIO (templates em `auxiliary_files/`):
`kalibr_maplab_config`, `kalibr_msf_config`, `kalibr_okvis_config`, `kalibr_rovio_config`.

> **Integração com o AQUA-SLAM:** não há exportador para o formato do AQUA-SLAM/ORB-SLAM3. A conversão
> das saídas (`camchain-imucam.yaml`, `imu.yaml`) para o YAML do AQUA-SLAM é justamente o trabalho de
> integração futuro — ver `domain/cam-imu-calibration.md`.
