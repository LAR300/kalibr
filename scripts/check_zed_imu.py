#!/usr/bin/env python3
"""Verifica a pose camera <-> IMU-da-ZED contra o nominal do `zed_macro` (R8/CA6 da spec).

## Por que esta verificacao e' especial

Todas as outras comparacoes geometricas do projeto dependem do xacro do ROV, que **mudou** entre os
datasets (a IMU externa foi reposicionada no v3) e cujas cotas novas ainda nao temos. Esta nao
depende: a posicao da IMU interna da ZED em relacao a camera vem do `zed_macro.urdf.xacro`, do
proprio fabricante, e a camera e' o mesmo modelo em todas as estruturas.

E' portanto a **unica ancora geometrica confiavel** disponivel hoje.

## A geometria

O `zed_macro` coloca `zed_node_imu_link` a `[-0.002, -0.023, -0.002]` do
`zed_node_left_camera_frame` (frame do CORPO da camera: x=frente, y=esquerda, z=cima).

O Kalibr trabalha no frame OPTICO (x=direita, y=baixo, z=frente). A ligacao entre os dois, tambem
do `zed_macro`, e' `rpy = (-pi/2, 0, -pi/2)`, que da'

    R_corpo_optico = [[0,0,1],[-1,0,0],[0,-1,0]]

Entao o nominal no frame optico e' `R_corpo_optico^T @ [-0.002,-0.023,-0.002]`.

Do Kalibr, `T_ci` (imu0 -> cam0) mapeia `p_cam = R p_imu + t`; com `p_imu = 0` sai `p_cam = t`, ou
seja **a translacao de `T_ci` e' a origem da IMU no frame optico da camera** — exatamente o que
comparamos.

Uso:
  python3 check_zed_imu.py data/output/runs/v3_raw__zed-fabrica ...
  python3 check_zed_imu.py --auto      # todos os runs com IMU da ZED
"""
from __future__ import print_function
import argparse
import glob
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from compare_calibrations import parse_camchain, invert, rot_angle

# zed_macro.urdf.xacro: left_camera_frame -> imu_link
P_IMU_NO_CORPO = np.array([-0.002, -0.023, -0.002])
R_CORPO_OPTICO = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], dtype=float)
P_NOMINAL_OPTICO = R_CORPO_OPTICO.T @ P_IMU_NO_CORPO


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("runs", nargs="*")
    ap.add_argument("--auto", action="store_true",
                    help="Varre data/output/runs/ por execucoes com a IMU da ZED.")
    a = ap.parse_args()

    runs = list(a.runs)
    if a.auto or not runs:
        here = os.path.dirname(os.path.abspath(__file__))
        runs = sorted(d for d in glob.glob(os.path.join(here, "data/output/runs/*"))
                      if "__zed-" in os.path.basename(d))

    print("=" * 96)
    print("VERIFICACAO GEOMETRICA INDEPENDENTE — camera <-> IMU da ZED (R8/CA6)")
    print("=" * 96)
    print("Nominal do zed_macro, no frame OPTICO da camera esquerda:")
    print("   %s m   (|.| = %.4f m = %.1f mm)"
          % (np.array2string(P_NOMINAL_OPTICO, precision=4), np.linalg.norm(P_NOMINAL_OPTICO),
             1000 * np.linalg.norm(P_NOMINAL_OPTICO)))
    print("Nao depende do xacro do ROV — vem do modelo do sensor.\n")

    print("  %-34s %-26s %9s %9s" % ("execucao", "t_ci medido [x y z]", "|t| mm", "erro mm"))
    print("  " + "-" * 82)
    vals = []
    for d in runs:
        nome = os.path.basename(d)
        cc = glob.glob(os.path.join(d, "*-camchain-imucam.yaml"))
        if not cc:
            print("  %-34s (sem resultado)" % nome)
            continue
        cams = parse_camchain(cc[0])
        if 0 not in cams:
            continue
        t = cams[0]["T"][:3, 3]          # T_ci: origem da IMU no frame optico da cam0
        err = np.linalg.norm(t - P_NOMINAL_OPTICO)
        vals.append(t)
        print("  %-34s %-26s %9.1f %9.1f"
              % (nome, np.array2string(t, precision=4), 1000 * np.linalg.norm(t), 1000 * err))

    if len(vals) > 1:
        V = np.array(vals)
        print("\n  dispersao entre execucoes: %s mm  (norma 3D %.1f mm)"
              % (np.array2string(1000 * V.std(axis=0), precision=1),
                 1000 * float(np.linalg.norm(V.std(axis=0)))))
        d_med = V.mean(axis=0) - P_NOMINAL_OPTICO
        print("  vies medio vs. zed_macro : %s mm  (norma %.1f mm)"
              % (np.array2string(1000 * d_med, precision=1), 1000 * float(np.linalg.norm(d_med))))
        print("\n  Leitura: o zed_macro e' nominal de fabricante (nao medido por unidade), entao")
        print("  alguns mm de desvio sao esperados. Dispersao >> vies significaria que o problema")
        print("  e' a estimacao; vies >> dispersao, que o nominal e' que nao descreve esta unidade.")


if __name__ == "__main__":
    main()
