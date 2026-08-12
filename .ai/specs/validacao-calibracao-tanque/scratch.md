# Scratch — Validação (notas de processo, efêmero)

## Insumos coletados (bag 01)
- Câmera (ZED, `config/sensors/zed2i01/zed_opencv_calibration.yaml`): 1280×720, baseline 0.1211 m,
  radtan 5-params (k3 grande, dropado — D7). Camchain em `data/.../config01/camchain.yaml`.
- Xacro (`config/xacro/petro_rov.urdf.xacro`) — posições no base_link (m):
  ZED cam `[0.133,0,-0.015]`, ZED IMU `[-0.002,-0.023,-0.002]` (rel. left cam),
  DVL `[0,0,-0.13992]`, Microstrain `[-0.09439,0,-0.01112]`.
  - Guia (translação): lever-arm Microstrain→DVL ≈ `[0.094, 0, -0.129]`; câmera→Microstrain ≈ `[0.227, 0, -0.004]`.
- Bag 01: imagens 1280×720 **bgra8**; tópicos `/zed/zed_node/{left,right}/color/raw/image`,
  `/imu/data` (~197 Hz, Microstrain), `/zed/zed_node/imu/data` (~98 Hz, ZED), `/dvl/data` (~9 Hz).

## Decisões desta rodada (confirmadas com o usuário)
- **sound_speed = 1500 m/s** (default) — usuário confirmou usar o padrão e **verificar depois** (REAVALIAR).
- **Microstrain = 3DM-GV7-AHRS** — valores de ruído PROVISÓRIOS (datasheet/típicos) no `config01/imu.yaml`
  (REAVALIAR com Allan variance).
- **1º run com Microstrain-só** (IMU de referência) para reduzir variáveis; **adicionar a ZED IMU depois**
  (fallback previsto na D5).

## Ainda a confirmar
- **Rotação IMU↔DVL / Microstrain 180°Z:** o xacro comenta "180° em Z" mas o joint tem `rpy=0 0 0`.
  Ambiguidade de rotação — o `T_dvl_imu` inicial do `dvl0.yaml` começa identidade (a ferramenta estima). (REAVALIAR)

## Estado (retomar amanhã) — bag 01
- ✅ Prep: `data/output/piscina_calib_01.bag` (mono8, rebaseado, T0_NS=1785350248197642944) +
  `dvl0_calib01.csv` (rebaseado com o mesmo t0). mcap extraído já apagado (zip preservado).
- ✅ Cam-IMU concluído: `data/output/piscina_calib_01-camchain-imucam.yaml` + `-imu.yaml`.
  Reprojeção ~1.5 px (k3), accel 0.036 / giro 0.0063, gravidade 9.807, timeshift 16.8 ms.
- ⏭️ **PRÓXIMO (amanhã): DVL** — `kalibr_calibrate_dvl` no bag 01, reusando o camchain-imucam acima:
  ```
  rosrun kalibr kalibr_calibrate_dvl \
    --bag /data/output/piscina_calib_01.bag \
    --cams /data/output/piscina_calib_01-camchain-imucam.yaml \
    --imu  /data/output/piscina_calib_01-imu.yaml \
    --target /data/calibration/cam_imu_dvl/config01/target.yaml \
    --dvl  /data/calibration/cam_imu_dvl/config01/dvl0.yaml \
    --dvl-csv /data/calibration/cam_imu_dvl/dvl0_calib01.csv \
    --timeoffset-padding 0.1 --dont-show-report
  ```
- Comando padrão do cam-IMU (para os outros bags): mesmo do 3.1 **com `--timeoffset-padding 0.1`**.

## Riscos ativos
- **`--timeoffset-padding 0.1` obrigatório:** o timeshift real câmera-IMU (~56 ms prior, 16.8 ms final)
  excede o padrão de 30 ms → "Spline Buffer Exceeded". Usar 0.1 em todos os runs (cam-IMU e DVL).
- **encoding bgra8:** o Kalibr pode precisar de bgr8/mono8. Se falhar no passo 3.1, converter o encoding
  na conversão ROS 2→ROS 1 (ou preprocessar). cv_bridge normalmente lida com bgra8→mono8.
- **k3 dropado (D7):** reprojeção pode subir; TODO futuro = calibrar intrínsecos com o Kalibr.

## TODO futuro (fora desta spec)
- Allan variance das IMUs (spec própria).
- Calibrar intrínsecos com o Kalibr (pinhole-radtan/equi) e comparar com a ZED-4params.
