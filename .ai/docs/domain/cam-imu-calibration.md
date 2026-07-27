---
description: Fluxo de calibração câmera+IMU do Kalibr (caso ZED 2i deste fork) e como consumir as saídas em sistemas VIO/SLAM. Abrir ao rodar uma calibração ou ao integrar os resultados noutro sistema.
sources: [scripts/docs/kalibr.md, scripts/config/, scripts/Makefile, aslam_offline_calibration/kalibr/python/kalibr_calibrate_imu_camera]
---

# Calibração câmera + IMU (fluxo e integração)

> Regra de negócio central deste fork. O guia operacional completo (com troubleshooting) está em
> `scripts/docs/kalibr.md`; aqui fica o essencial + a ponte de integração.

## Objetivo
Estimar, para um par câmera+IMU (aqui, a **ZED 2i**): os **intrínsecos** da câmera, a transformação
espacial **`T_cam_imu`** (IMU→câmera) e o **`timeshift_cam_imu`** (offset de relógio). Sem isso, VIO/SLAM
(VINS, MSCKF, ORB-SLAM3, AQUA-SLAM) não fundem os sensores corretamente.

## Fluxo (via `scripts/`, ambiente Docker 20.04/Noetic)
1. **Ambiente:** `cd scripts && make build && make run` (monta `scripts/data` → `/data`; X11 para os reports). `roscore` no host ou no container.
2. **Dados brutos** em `data/calibration/cam_imu/<SESSAO>/`: `cam0/` (imagens `<ts_ns>.png`), `imu0.csv` (7 colunas), `camchain.yaml`, `imu0.yaml`, `target.yaml`.
3. **Gerar o bag:** `rosrun kalibr kalibr_bagcreater --folder /data/.../<SESSAO> --output-bag /data/output/cam_imu.bag`.
4. **(Recomendado) Intrínsecos primeiro:** `rosrun kalibr kalibr_calibrate_cameras --bag ... --topics /cam0/image_raw --models pinhole-radtan --target .../target.yaml`. Gera um `camchain-*.yaml` melhor que valores de fabricante.
5. **CAM-IMU:** `rosrun kalibr kalibr_calibrate_imu_camera --bag ... --cams camchain.yaml --imu imu0.yaml --target target.yaml`. Saídas: `*-camchain-imucam.yaml`, `*-imu.yaml`, `*-results-imucam.txt`, `*-report-imucam.pdf`.

## Boas práticas que afetam o resultado
- Mover o sensor nos **6 graus de liberdade** por 60–120 s; alvo fixo, sensor em movimento; evitar motion blur; cobrir todo o FOV.
- `resolution` e `rostopic` do YAML **devem** casar com os dados reais (senão a extração falha).
- Medir `tagSize` com precisão (define a **escala**). Ruído da IMU realista (Allan Variance de preferência).
- Qualidade: erro de reprojeção <0.5px excelente; translação de `T_cam_imu` deve bater com a distância física; timeshift <10ms para sync por hardware; gravidade ≈ 9.81.

## Ponte de integração: Kalibr → AQUA-SLAM

Este é o objetivo da integração futura. O Kalibr **produz** os parâmetros que o AQUA-SLAM (`ORB_DVL2`)
**consome** — mas em formatos e convenções diferentes; é preciso converter.

| Saída do Kalibr (`camchain-imucam.yaml` / `imu.yaml`) | Destino no AQUA-SLAM (`data/*.yaml`, OpenCV FileStorage) |
|---|---|
| `intrinsics: [fx, fy, cx, cy]` | `Camera.fx/fy/cx/cy` |
| `distortion_coeffs` (radtan) | `Camera.k1/k2/p1/p2` (conferir ordem) |
| baseline estéreo (do extrínseco entre câmeras) × `fx` | `Camera.bf` (derivar) |
| `T_cam_imu` (4×4) | `T_gyro_c` (4×4, IMU→câmera) — **conferir direção/inversão** |
| `gyroscope_noise_density` / `accelerometer_noise_density` | `IMU.NoiseGyro` / `IMU.NoiseAcc` |
| `gyroscope_random_walk` / `accelerometer_random_walk` | `IMU.GyroWalk` / `IMU.AccWalk` |
| `update_rate` | `IMU.Frequency` |
| `timeshift_cam_imu` | **sem campo** no AQUA-SLAM (avaliar) |

Diferenças a tratar na conversão (ver o lado do AQUA-SLAM em `AQUA-SLAM/.ai/docs/domain/sensor-fusion.md`):
- **Formato:** Kalibr usa YAML padrão; AQUA-SLAM usa OpenCV FileStorage (`%YAML:1.0`, `!!opencv-matrix`). Incompatíveis — precisa de um conversor.
- **Convenção de `T_cam_imu` vs `T_gyro_c`:** direção (source→target) pode exigir **inversão**; validar.
- **DVL fora do escopo:** Kalibr não calibra DVL (`T_dvl_c`, `alpha`, `beta`) — isso o AQUA-SLAM faz online.
- **Não há exportador pronto** (`python/exporters/` cobre maplab/msf/okvis/rovio, não AQUA-SLAM). O conversor é código novo a escrever na integração.
