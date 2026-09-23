#!/usr/bin/env bash
# Matriz de calibracao camera-IMU do dataset v3 (fora d'agua).
#
#   bag  : raw   (imagens nao retificadas)  |  rect (retificadas pelo SDK)
#   imu  : micro (MicroStrain externa)      |  zed  (IMU interna da ZED)
#   intr : fabrica (camera_info do SDK)     |  kalibr (estimados na Fase 3)
#
# Nome do run: v3_<bag>__<imu>-<intr>  -> data/output/runs/<nome>/
# O symlink para o bag leva o nome do run, entao o prefixo de TODAS as saidas do Kalibr
# vira o rotulo da variante (ver data/output/README.md e D8).
#
# Sempre com --recompute-camera-chain-extrinsics (D9): o T_cn_cnm1 derivado do campo R do
# camera_info tinha 0.389 deg de erro (= 6.5 px), que congelado inflava a reprojecao em 4.8x.
# O camchain fornece apenas o chute inicial; o Kalibr estima o extrinseco estereo.
#
# Uso:
#   ./run_v3_matriz.sh raw:zed rect:micro rect:zed          # fase 2 (intr=fabrica)
#   INTR=kalibr ./run_v3_matriz.sh raw:micro raw:zed ...    # fase 4
set -uo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
OUT="$HERE/data/output"; BAGS="$OUT/bags"; RUNS="$OUT/runs"
CONTAINER=${CONTAINER:-kalibr_zed}
PADDING=${PADDING:-0.3}
INTR=${INTR:-fabrica}

camchain_raw_fabrica() {  # K + radtan AJUSTADO ao rational (rational_to_radtan.py)
cat <<'YAML'
# Bag v3 RAW. Intrinsecos de FABRICA (camera_info do SDK).
# distortion_coeffs NAO sao os 4 primeiros do rational_polynomial: truncar daria 989% de erro
# na borda. Sao um AJUSTE por minimos quadrados que reproduz o mapeamento racional
# (scripts/rational_to_radtan.py), residuo 0.16 px medio.
# T_cn_cnm1 aqui e' so' CHUTE INICIAL -- o Kalibr reestima (D9).
cam0:
  camera_model: pinhole
  intrinsics: [957.790, 958.185, 640.875, 354.266]
  distortion_model: radtan
  distortion_coeffs: [-0.06989657, -0.04761953, 0.00048393, 0.00000364]
  resolution: [1280, 720]
  rostopic: /zed/zed_node/left/gray/raw/image
  cam_overlaps: [1]
cam1:
  camera_model: pinhole
  intrinsics: [957.750, 958.140, 650.260, 360.758]
  distortion_model: radtan
  distortion_coeffs: [-0.06829699, -0.04969007, -0.00048094, 0.00008285]
  resolution: [1280, 720]
  rostopic: /zed/zed_node/right/gray/raw/image
  cam_overlaps: [0]
  T_cn_cnm1:
  - [0.9999830000, -0.0015340000, 0.0057120000, -0.1200909584]
  - [0.0015210000, 0.9999960000, 0.0022370000, -0.0001826615]
  - [-0.0057150000, -0.0022290000, 0.9999810000, 0.0006863315]
  - [0.0000000000, 0.0000000000, 0.0000000000, 1.0000000000]
YAML
}

camchain_rect_fabrica() {  # P + distorcao ZERADA + rotacao identidade (D3)
cat <<'YAML'
# Bag v3 RECT (imagens ja retificadas pelo SDK).
# Intrinsecos da matriz P; distorcao ZERADA -- aplicar D numa imagem retificada corrige duas
# vezes e enviesa em silencio. As duas cameras tem intrinsecos identicos, como esperado de um
# par retificado. Baseline -P[3]/fx = 0.120088 m, rotacao identidade por construcao (D3).
# T_cn_cnm1 e' chute inicial; o Kalibr reestima (D9) -- e o quanto ele se mover mede a
# qualidade da retificacao do SDK.
cam0:
  camera_model: pinhole
  intrinsics: [947.799, 947.799, 647.908, 357.195]
  distortion_model: radtan
  distortion_coeffs: [0.0, 0.0, 0.0, 0.0]
  resolution: [1280, 720]
  rostopic: /zed/zed_node/left/gray/rect/image
  cam_overlaps: [1]
cam1:
  camera_model: pinhole
  intrinsics: [947.799, 947.799, 647.908, 357.195]
  distortion_model: radtan
  distortion_coeffs: [0.0, 0.0, 0.0, 0.0]
  resolution: [1280, 720]
  rostopic: /zed/zed_node/right/gray/rect/image
  cam_overlaps: [0]
  T_cn_cnm1:
  - [1.0, 0.0, 0.0, -0.120088]
  - [0.0, 1.0, 0.0, 0.0]
  - [0.0, 0.0, 1.0, 0.0]
  - [0.0, 0.0, 0.0, 1.0]
YAML
}

imu_micro() {
cat <<'YAML'
# IMU EXTERNA de referencia: MicroStrain 3DM-GV7-AHRS. Continuidade: 0 gaps, 0.0 s perdidos.
# Ruido PROVISORIO de datasheet - reavaliar com Allan variance (D6).
rostopic: /imu/data
update_rate: 200
model: calibrated
accelerometer_noise_density: 0.002
accelerometer_random_walk:   0.00004
gyroscope_noise_density:     0.0002
gyroscope_random_walk:       0.000002
YAML
}

imu_zed() {
cat <<'YAML'
# IMU INTERNA da ZED 2i (MEMS de consumo). Taxa medida ~99 Hz; 0-3 gaps, <=0.4 s perdidos.
# Ruido PROVISORIO, INFLADO ~5x em relacao a MicroStrain (D6): subestimar faz o otimizador
# confiar demais na IMU e brigar com os termos de camera; inflar e' o erro seguro.
# Sem Allan variance para nenhuma das duas IMUs.
rostopic: /zed/zed_node/imu/data
update_rate: 99
model: calibrated
accelerometer_noise_density: 0.01
accelerometer_random_walk:   0.0004
gyroscope_noise_density:     0.001
gyroscope_random_walk:       0.00002
YAML
}

target() {
cat <<'YAML'
# AprilGrid. tagSize ainda NAO medido fisicamente (premissa aberta da spec).
target_type: 'aprilgrid'
tagCols: 6
tagRows: 6
tagSize: 0.088
tagSpacing: 0.3
YAML
}

for SPEC in "$@"; do
  BAG="${SPEC%%:*}"; IMU="${SPEC##*:}"
  NAME="v3_${BAG}__${IMU}-${INTR}"
  D="$RUNS/$NAME"
  mkdir -p "$D/config"

  case "${BAG}_${INTR}" in
    raw_fabrica)  camchain_raw_fabrica  > "$D/config/camchain.yaml" ;;
    rect_fabrica) camchain_rect_fabrica > "$D/config/camchain.yaml" ;;
    *_kalibr)
      SRC="$OUT/intrinsecos/camchain_${BAG}_kalibr.yaml"
      if [ ! -f "$SRC" ]; then echo "[$NAME] FALTA $SRC (rode a Fase 3)"; continue; fi
      cp "$SRC" "$D/config/camchain.yaml" ;;
    *) echo "[$NAME] combinacao desconhecida"; continue ;;
  esac
  case "$IMU" in
    micro) imu_micro > "$D/config/imu.yaml" ;;
    zed)   imu_zed   > "$D/config/imu.yaml" ;;
    *) echo "[$NAME] IMU desconhecida: $IMU"; continue ;;
  esac
  target > "$D/config/target.yaml"
  ln -sf "../../bags/v3_${BAG}.bag" "$D/$NAME.bag"

  echo "############ $NAME ############"
  docker exec "$CONTAINER" bash -c "
    source /opt/ros/noetic/setup.bash
    source /catkin_ws/devel/setup.bash
    export MPLBACKEND=Agg
    rosrun kalibr kalibr_calibrate_imu_camera \
      --bag /data/output/runs/$NAME/$NAME.bag \
      --cams /data/output/runs/$NAME/config/camchain.yaml \
      --imu  /data/output/runs/$NAME/config/imu.yaml \
      --target /data/output/runs/$NAME/config/target.yaml \
      --timeoffset-padding $PADDING --recompute-camera-chain-extrinsics --dont-show-report
  " > "$D/run.log" 2>&1
  RC=$?

  if [ -f "$D/$NAME-results-imucam.txt" ]; then
    echo "[$NAME] OK (exit=$RC)"
    grep -aE "Reprojection error \(cam[01]\)|Gyroscope error \(imu0\)|Accelerometer error \(imu0\)" \
         "$D/$NAME-results-imucam.txt" | head -4 | sed "s/^/[$NAME]   /"
    tr '\r' '\n' < "$D/run.log" | grep -oE "^\[[0-9]+\]: J: [0-9.e+]+.*lambda:[0-9.e-]+" | tail -1 | sed "s/^/[$NAME]   ultima iter: /"
  else
    echo "[$NAME] FALHOU (exit=$RC)"
    tr '\r' '\n' < "$D/run.log" | grep -aoE "Optimization failed|RuntimeError.*|Spline.*Exceeded|Could not find topic.*" | tail -3 | sed "s/^/[$NAME]   /"
  fi
done
echo "### MATRIZ FINALIZADA"
