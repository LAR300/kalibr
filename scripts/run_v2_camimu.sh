#!/usr/bin/env bash
# Pipeline completo camera + IMU externa (Microstrain) para o dataset v2.
#
# Layout: bags em bags/ (uma copia), resultados em runs/<bag>__<variante>/, ligados por symlink.
#
#   data/output/bags/v2_<id>.bag              <- uma copia so, compartilhada por symlink
#   data/output/bags/v2_<id>-dvl.csv         <- mesmo t0 do bag
#   data/output/runs/v2_<id>__<VARIANTE>/
#     ├── v2_<id>__<VARIANTE>.bag -> ../../bags/v2_<id>.bag
#     ├── config/{camchain,imu,target}.yaml  <- config exata usada
#     ├── cmd.txt                            <- comando + desfecho
#     ├── v2_<id>__<VARIANTE>-*imucam.*      <- saidas do Kalibr (prefixo = nome do symlink)
#     └── run.log
#
# O symlink e' o truque: o Kalibr grava ao lado do bag usando a STRING do caminho
# (kalibr_calibrate_imu_camera:218), entao o nome do link vira o prefixo das saidas
# e a variante se auto-rotula. Ver data/output/README.md.
#
# Uso:  ./run_v2_camimu.sh [id ...]     (sem argumentos: todos os 3)
set -uo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
SRC="$HERE/data/calibration/cam_imu_dvl_v2"
OUT="$HERE/data/output"
BAGS="$OUT/bags"
RUNS="$OUT/runs"
CONTAINER=${CONTAINER:-kalibr_zed}
LEFT=/zed/zed_node/left/color/raw/image
RIGHT=/zed/zed_node/right/color/raw/image

# Janela em que o timeshift camera-IMU pode deslizar durante a otimizacao [s].
# Se o offset real exceder isso -> "Spline Coefficient Buffer Exceeded" e a otimizacao morre.
# v1 precisou de 0.1 (padrao 0.03 estourava). No v2 os priors sao MAIORES -- 95 ms (cam0) e
# 130 ms (cam1) no bag 11-39-12 -- e 0.1 tambem estourou. Dai 0.3, com folga.
PADDING=${PADDING:-0.3}

# Rotulo da variante -> vira o nome da pasta e o prefixo das saidas.
VARIANTE=${VARIANTE:-pad$PADDING}
# Flags extras do kalibr (ex.: EXTRA="--recompute-camera-chain-extrinsics")
EXTRA=${EXTRA:-}

IDS=("$@")
if [ ${#IDS[@]} -eq 0 ]; then
  IDS=(2026-09-18_11-39-12_calib 2026-09-18_11-44-41_calib 2026-09-18_11-50-20_calib)
fi

# taxa da IMU medida por bag (msgs/duracao) - so' para registro no yaml
imu_rate_of() {
  case "$1" in
    *11-39-12*) echo "199.9" ;;
    *11-44-41*) echo "198.3" ;;
    *11-50-20*) echo "200.0" ;;
    *)          echo "200.0" ;;
  esac
}

gerar_config() {   # $1 = dir de config, $2 = id do bag, $3 = taxa medida
  mkdir -p "$1"
  cat > "$1/camchain.yaml" <<YAML
# Config do bag $2 (dataset v2).
# Intrinsecos: calibracao submersa da ZED (zed_opencv_calibration.yaml), radtan 4-params
# com k3 DROPADO (D7) -> reprojecao esperada ~1.3-1.5 px. Referencia do SDK (ar):
# fx=957.79 cx=640.88 -> razao agua/ar ~1.43 (refracao). Recalibrar os intrinsecos e' a Fase 3.
cam0:
  camera_model: pinhole
  intrinsics: [1372.1188778, 1352.8871436, 615.33332373, 378.28765019]
  distortion_model: radtan
  distortion_coeffs: [0.28288749816, 0.48880552341, 0.027467750757, -0.0083743777485]
  resolution: [1280, 720]
  rostopic: $LEFT
  cam_overlaps: [1]
cam1:
  camera_model: pinhole
  intrinsics: [1372.0764618, 1354.5967480, 632.82131409, 388.73086264]
  distortion_model: radtan
  distortion_coeffs: [0.26654473557, 0.79808049630, 0.027388112907, -0.0046617579910]
  resolution: [1280, 720]
  rostopic: $RIGHT
  cam_overlaps: [0]
  T_cn_cnm1:
  - [0.9999888957, -0.0022497174, -0.0041409229, -0.1210557465]
  - [0.0022225140, 0.9999759978, -0.0065623318, -0.0003672597]
  - [0.0041555869, 0.0065530557, 0.9999698938, 0.0018001661]
  - [0.0000000000, 0.0000000000, 0.0000000000, 1.0000000000]
YAML
  cat > "$1/imu.yaml" <<YAML
# IMU EXTERNA de referencia: MicroStrain 3DM-GV7-AHRS. Bag $2 (dataset v2).
# Taxa medida neste bag: $3 Hz (sem gaps relevantes: perda de 0.0 s).
# Ruido: valores PROVISORIOS de datasheet - reavaliar com Allan variance (D3).
rostopic: /imu/data
update_rate: 200
model: calibrated
accelerometer_noise_density: 0.002     # m/s^2/sqrt(Hz)  (provisorio)
accelerometer_random_walk:   0.00004   # m/s^3/sqrt(Hz)  (provisorio)
gyroscope_noise_density:     0.0002    # rad/s/sqrt(Hz)  (provisorio)
gyroscope_random_walk:       0.000002  # rad/s^2/sqrt(Hz)(provisorio)
YAML
  cat > "$1/target.yaml" <<YAML
# AprilGrid. ATENCAO: tagSize ainda NAO foi medido fisicamente (premissa aberta).
# Define a escala metrica de tudo -> erro de 1% vira 1% em todas as translacoes.
target_type: 'aprilgrid'
tagCols: 6
tagRows: 6
tagSize: 0.088        # metros  (A MEDIR com paquimetro, molhado)
tagSpacing: 0.3       # razao espacamento/tamanho
YAML
}

for ID in "${IDS[@]}"; do
  SHORT=$(echo "$ID" | sed -E 's/.*_([0-9]{2}-[0-9]{2}-[0-9]{2})_calib/\1/')
  BAGNAME="v2_$SHORT"
  NAME="${BAGNAME}__${VARIANTE}"
  D="$RUNS/$NAME"
  BAG="$BAGS/$BAGNAME.bag"
  LINK="$D/$NAME.bag"
  LOG="$D/run.log"
  mkdir -p "$D" "$BAGS"

  echo "############ $SHORT ############"
  gerar_config "$D/config" "$ID" "$(imu_rate_of "$ID")"
  echo "[$SHORT] config gerada em $D/config/"

  if [ ! -f "$BAG" ]; then
    echo "[$SHORT] 1/3 convertendo ROS 2 -> ROS 1 (mono8, rebasing)..."
    python3 "$HERE/ros2_to_ros1_kalibr.py" "$SRC/$ID" -o "$BAG" \
      --image-topics "$LEFT" "$RIGHT" --imu-topics /imu/data > "$BAGS/$BAGNAME-prep.log" 2>&1
    if [ ! -f "$BAG" ]; then echo "[$SHORT] ERRO na conversao:"; tail -5 "$BAGS/$BAGNAME-prep.log"; continue; fi
  else
    echo "[$SHORT] 1/3 bag ROS 1 ja existe, pulando conversao"
  fi
  T0=$(grep -oP 'T0_NS=\K[0-9]+' "$BAGS/$BAGNAME-prep.log" | head -1)
  echo "[$SHORT]     T0_NS=$T0"

  if [ ! -f "$BAGS/$BAGNAME-dvl.csv" ] && [ -n "$T0" ]; then
    echo "[$SHORT] 2/3 extraindo CSV do DVL (mesmo t0)..."
    python3 "$HERE/ros2_dvl_to_csv.py" "$SRC/$ID" -o "$BAGS/$BAGNAME-dvl.csv" --t0-ns "$T0" \
      >> "$BAGS/$BAGNAME-prep.log" 2>&1
    echo "[$SHORT]     $(wc -l < "$BAGS/$BAGNAME-dvl.csv") linhas"
  fi

  ln -sf "../../bags/$BAGNAME.bag" "$LINK"
  echo "[$SHORT] 3/3 kalibr_calibrate_imu_camera (variante=$VARIANTE, padding=$PADDING)..."
  docker exec "$CONTAINER" bash -c "
    source /opt/ros/noetic/setup.bash
    source /catkin_ws/devel/setup.bash
    export MPLBACKEND=Agg
    rosrun kalibr kalibr_calibrate_imu_camera \
      --bag /data/output/runs/$NAME/$NAME.bag \
      --cams /data/output/runs/$NAME/config/camchain.yaml \
      --imu  /data/output/runs/$NAME/config/imu.yaml \
      --target /data/output/runs/$NAME/config/target.yaml \
      --timeoffset-padding $PADDING $EXTRA --dont-show-report
  " > "$LOG" 2>&1
  RC=$?

  if [ -f "$D/$NAME-results-imucam.txt" ]; then
    echo "[$SHORT] OK (exit=$RC)"
    grep -aE "Reprojection error \(cam[01]\) \[px\]|Gyroscope error|Accelerometer error" \
         "$D/$NAME-results-imucam.txt" | sed "s/^/[$SHORT]   /"
  else
    echo "[$SHORT] FALHOU (exit=$RC)"
    grep -aoE "Optimization failed|RuntimeError.*|Spline.*Exceeded|Could not find topic.*" "$LOG" | tail -3 | sed "s/^/[$SHORT]   /"
  fi
done

echo "############ COMPARACAO ############"
python3 "$HERE/compare_calibrations.py" --outdir "$RUNS" | tee "$OUT/analises/comparacao-crossbag.txt"
echo "### PIPELINE V2 FINALIZADO"
