#!/usr/bin/env bash
# Fase 3 da spec calibracao-cam-imu-ar: estimar os intrinsecos com o proprio Kalibr.
#
# Serve a dois propositos:
#   1. dar a fonte "kalibr" de intrinsecos para a Fase 4 (R3b);
#   2. DIAGNOSTICAR o bag rect. Se a distorcao estimada sair ~0 e fx ~947.8, o camchain que
#      montei do camera_info esta' certo e o mal-condicionamento tem outra causa. Se sair
#      diferente, a retificacao do SDK nao corresponde ao camera_info do topico rect.
#
# Mesma armadilha de sempre: o Kalibr grava ao lado do bag -> symlink por execucao (D8).
set -uo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
OUT="$HERE/data/output"; RUNS="$OUT/runs"; INTR="$OUT/intrinsecos"
CONTAINER=${CONTAINER:-kalibr_zed}
mkdir -p "$INTR"

target_yaml() {
cat <<'YAML'
target_type: 'aprilgrid'
tagCols: 6
tagRows: 6
tagSize: 0.088
tagSpacing: 0.3
YAML
}

# bag  topico_esq  topico_dir
run_intr() {
  local BAG=$1 L=$2 R=$3 MODELO=${4:-pinhole-radtan}
  local NAME="v3_${BAG}__intrinsecos-${MODELO##*-}"
  local D="$RUNS/$NAME"
  mkdir -p "$D/config"
  target_yaml > "$D/config/target.yaml"
  ln -sf "../../bags/v3_${BAG}.bag" "$D/$NAME.bag"

  echo "############ $NAME ($MODELO) ############"
  docker exec "$CONTAINER" bash -c "
    source /opt/ros/noetic/setup.bash
    source /catkin_ws/devel/setup.bash
    export MPLBACKEND=Agg
    rosrun kalibr kalibr_calibrate_cameras \
      --bag /data/output/runs/$NAME/$NAME.bag \
      --topics $L $R \
      --models $MODELO $MODELO \
      --target /data/output/runs/$NAME/config/target.yaml \
      --dont-show-report
  " > "$D/run.log" 2>&1
  local RC=$?

  if [ -f "$D/$NAME-camchain.yaml" ]; then
    echo "[$NAME] OK (exit=$RC)"
    grep -aE "intrinsics|distortion_coeffs" "$D/$NAME-camchain.yaml" | sed "s/^/[$NAME]   /"
    tr '\r' '\n' < "$D/run.log" | grep -aE "Reprojection error.*mean" | head -2 | sed "s/^/[$NAME]   /"
    # publica para a Fase 4 (so' o radtan, que e' o modelo que o resto da matriz usa)
    if [ "$MODELO" = "pinhole-radtan" ]; then
      cp "$D/$NAME-camchain.yaml" "$INTR/camchain_${BAG}_kalibr.yaml"
      echo "[$NAME]   -> publicado em intrinsecos/camchain_${BAG}_kalibr.yaml"
    fi
  else
    echo "[$NAME] FALHOU (exit=$RC)"
    tr '\r' '\n' < "$D/run.log" | grep -aoE "Optimization failed|RuntimeError.*|Could not find topic.*|error.*" | tail -3 | sed "s/^/[$NAME]   /"
  fi
}

for arg in "$@"; do
  case "$arg" in
    raw)  run_intr raw  /zed/zed_node/left/gray/raw/image  /zed/zed_node/right/gray/raw/image ;;
    rect) run_intr rect /zed/zed_node/left/gray/rect/image /zed/zed_node/right/gray/rect/image ;;
    raw-equi)  run_intr raw  /zed/zed_node/left/gray/raw/image  /zed/zed_node/right/gray/raw/image pinhole-equi ;;
    rect-equi) run_intr rect /zed/zed_node/left/gray/rect/image /zed/zed_node/right/gray/rect/image pinhole-equi ;;
    *) echo "alvo desconhecido: $arg" ;;
  esac
done
echo "### INTRINSECOS FINALIZADOS"
