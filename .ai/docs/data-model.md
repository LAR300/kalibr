---
description: Formatos de arquivo do Kalibr — YAMLs de entrada (câmera, IMU, target) e de saída (camchain-imucam), CSV de IMU e nomes de imagem. Abrir ao criar/editar configs ou consumir os resultados. (Kalibr não tem banco; persistência é em arquivos.)
sources: [aslam_offline_calibration/kalibr/python/kalibr_common/ConfigReader.py, scripts/config/cam0.yaml, scripts/config/camchain.yaml, scripts/config/imu0.yaml, scripts/config/target.yaml, scripts/docs/kalibr.md]
---

# Modelo de dados (arquivos)

> Kalibr não tem banco. Toda persistência é em **arquivos**: YAML de config/resultado, ROS bag,
> CSV de IMU, PDF de report. Os schemas vivem em `python/kalibr_common/ConfigReader.py`
> (`CameraParameters`, `ImuParameters`, `CameraChainParameters`, `CalibrationTargetParameters`).
> Exemplos reais em `scripts/config/`.

## Entrada — câmera (`camchain.yaml` / `cam0.yaml`)
```yaml
cam0:
  camera_model: pinhole            # pinhole | omni | ds (double sphere) | eucm
  intrinsics: [fx, fy, cx, cy]     # depende do modelo (omni: [xi,fx,fy,cx,cy]; ds: [xi,alpha,fx,fy,cx,cy])
  distortion_model: radtan         # radtan | equidistant | fov | none
  distortion_coeffs: [k1, k2, p1, p2]
  resolution: [largura, altura]
  rostopic: /cam0/image_raw        # deve casar com o tópico no bag: /<nome_pasta>/image_raw
  cam_overlaps: []                 # índices de câmeras com FOV sobreposto
# multi-câmera acrescenta:  T_cn_cnm1: <matriz 4x4>  (extrínseco câmera n ← n-1)
```

## Entrada — IMU (`imu0.yaml`)
```yaml
accelerometer_noise_density: 0.01     # m/s²/√Hz
accelerometer_random_walk: 0.0002     # m/s³/√Hz
gyroscope_noise_density: 0.0004       # rad/s/√Hz
gyroscope_random_walk: 0.00002        # rad/s²/√Hz
model: calibrated                     # calibrated | scale-misalignment | scale-misalignment-size-effect
rostopic: /imu0                       # deve casar com o tópico no bag: /<nome_csv_sem_extensao>
update_rate: 198                      # Hz
```
Valores de ruído: preferir **Allan Variance**; senão datasheet (o exemplo é BMI055 da ZED 2i); senão valores típicos.

## Entrada — target (`target.yaml`)
```yaml
# AprilGrid (recomendado)
target_type: 'aprilgrid'
tagCols: 6
tagRows: 6
tagSize: 0.088        # m (lado do quadrado preto do tag)
tagSpacing: 0.3       # razão espaçamento/tagSize
# --- ou checkerboard ---
# target_type: 'checkerboard'
# targetCols: 7 ; targetRows: 6 ; rowSpacingMeters: 0.03 ; colSpacingMeters: 0.03
# --- ou circlegrid ---
```

## Entrada — dados brutos (para o `bagcreater`)
- Imagens: pasta `cam0/`, arquivos nomeados `<timestamp_ns>.png|.jpg|.bmp`. O nome **é** o timestamp.
- IMU: `imu0.csv`, 7 colunas `timestamp[ns], omega_x, omega_y, omega_z [rad/s], alpha_x, alpha_y, alpha_z [m/s²]`.
- Nomes definem tópicos: `cam0/` → `/cam0/image_raw`; `imu0.csv` → `/imu0`.

## Saída — resultado CAM-IMU (`*-camchain-imucam.yaml`)
Acrescenta ao camchain:
```yaml
cam0:
  T_cam_imu:                    # 4x4, transformação IMU → câmera
  - [ ... ]
  timeshift_cam_imu: -0.001099  # s ; t_imu = t_cam + shift
  # + intrínsecos/distorção/resolution/rostopic já existentes
```
Também gera `*-imu.yaml` (parâmetros da IMU), `*-results-imucam.txt` (resumo textual) e `*-report-imucam.pdf`.

## DVL (extensão do fork — `kalibr_calibrate_dvl`)

**Config de entrada — `dvl0.yaml`** (YAML padrão; exemplo em `scripts/config/dvl0.yaml`):
```yaml
csv: /data/.../dvl0.csv        # fonte do stream do DVL (ou 'rostopic: /dvl/data')
update_rate: 8                 # Hz
sound_speed: 1500.0            # m/s configurado no A50 (referência p/ a escala)
velocity_scale: 1.0            # chute inicial da escala s
estimate_scale: true
estimate_time_offset: true
T_dvl_imu:                     # 4x4 SE3 inicial (IMU -> DVL), do xacro
  - [1.0, 0.0, 0.0, 0.0]
  - [0.0, 1.0, 0.0, 0.0]
  - [0.0, 0.0, 1.0, 0.0]
  - [0.0, 0.0, 0.0, 1.0]
velocity_noise_density: 0.02   # m/s (fallback quando a covariância do CSV é inválida)
gating: {require_velocity_valid: true, min_altitude: 0.1, max_altitude: 50.0, max_fom: 1.0, max_speed: 1.5}
```

**Stream de entrada — CSV do DVL** (16 colunas; exemplo/esquema em `scripts/config/dvl0_example.csv`),
extraído da msg `dvl_msgs/DVL` no lado ROS 2:
```
timestamp_ns, vx, vy, vz, cov00..cov22 (9), velocity_valid (0/1), fom, altitude
```

**Saída — `*-dvl-results.yaml`**:
```yaml
dvl0:
  T_dvl_imu: <4x4 SE3, IMU -> DVL>
  velocity_scale: <s>
  timeshift_dvl_imu: <s>
  sound_speed: <m/s>
```
Mais `*-results-dvl.txt` (RMS do resíduo de velocidade, por-eixo, nº usados/descartados) e `*-report-dvl.pdf`.

> **Convenção de `T_dvl_imu`:** SE3 padrão IMU→DVL na fronteira (config/saída); internamente parametrizado
> como rotação `C_dvl_b` + lever-arm `r_b` (ver `.ai/specs/dvl-calibration/decisions.md` D11).

## Invariantes / cuidados
- `resolution` no YAML **deve** ser igual à resolução real das imagens, senão a extração de cantos falha.
- `rostopic` **deve** casar com os tópicos gerados no bag (`/<pasta>/image_raw`, `/<csv>`).
- `tagSize`/espaçamentos definem a **escala** da calibração — erro de medida vira erro de escala.
- Passar ruído da IMU muito baixo faz o otimizador confiar demais na IMU; muito alto, de menos.
- Formato: YAML padrão (PyYAML). Não confundir com o formato **OpenCV FileStorage** (`%YAML:1.0`,
  `!!opencv-matrix`) usado pelo **AQUA-SLAM** — são incompatíveis; a integração precisa converter.

## Como interpretar resultados (referência rápida)
- Erro de reprojeção (mean): <0.5px excelente · 0.5–1.0 bom · 1.0–2.0 aceitável · >2.0 ruim.
- Translação de `T_cam_imu`: deve bater com a distância física câmera↔IMU (poucos cm na ZED 2i).
- Timeshift: <10ms normal (hardware sync); >50ms suspeito. Gravidade estimada ≈ 9.81 m/s².
