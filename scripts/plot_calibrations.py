#!/usr/bin/env python3
"""Vistas ortograficas das posicoes calibradas (por bag) vs. o nominal do xacro.

Complementa `compare_calibrations.py` (numeros) com o feedback visual da geometria.
Roda DENTRO do container kalibr (matplotlib):

  docker exec kalibr_zed bash -c '
    export MPLBACKEND=Agg
    python3 /catkin_ws/src/kalibr/scripts/plot_calibrations.py -o /data/output/geometria.png'

Duas figuras, porque a calibracao mede a posicao RELATIVA e nao distingue qual dos dois
corpos esta' fora do lugar:
  - ancorada na Microstrain (a IMU no lugar do xacro, as cameras se movem);
  - ancorada na camera    (a ZED no lugar do xacro, a Microstrain se move).
"""
from __future__ import print_function
import argparse
import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from compare_calibrations import (parse_camchain, parse_results, invert, rot_angle,
                                  P_IMU, P_DVL, P_CAM_NOM, P_ZED_CENTER, R_BASE_OPT)

# bag 01 = outlier (relogio nao assentado, D9) -> desenhado esmaecido
OUTLIERS = {"piscina_calib_01"}
CORES = {"piscina_calib_01": "#c0392b", "piscina_calib_02": "#2980b9",
         "piscina_calib_03": "#27ae60", "piscina_calib_04": "#8e44ad"}

VISTAS = [(0, 1, "Vista de cima (X-Y)", "x [m] (frente +)", "y [m] (esquerda +)"),
          (0, 2, "Vista lateral (X-Z)", "x [m] (frente +)", "z [m] (cima +)"),
          (1, 2, "Vista frontal (Y-Z)", "y [m] (esquerda +)", "z [m] (cima +)")]


def carregar(outdir):
    """{bag: {'R_base_imu':.., 'cams': {i: t_ic}, 'ts':.., 'rep':..}}"""
    out = {}
    for n in sorted(os.listdir(outdir)):
        if not n.endswith("-camchain-imucam.yaml"):
            continue
        bag = n.replace("-camchain-imucam.yaml", "")
        cams = parse_camchain(os.path.join(outdir, n))
        if 0 not in cams:
            continue
        out[bag] = {
            "R_base_imu": R_BASE_OPT @ cams[0]["T"][:3, :3],
            "cams": {i: invert(cams[i]["T"])[:3, 3] for i in cams},
            "ts": cams[0]["timeshift"],
            "rep": parse_results(os.path.join(outdir, bag + "-results-imucam.txt")).get(0),
        }
    return out


def desenhar(ax, ia, ib, titulo, xlab, ylab, dados, ancora):
    """ancora='imu': IMU no nominal, cameras se movem. 'cam': o inverso."""
    p = lambda v: (v[ia], v[ib])

    # --- nominal (xacro) ---
    ax.plot(*p(P_IMU), "ks", ms=11, mfc="none", mew=2, zorder=5)
    ax.annotate("Microstrain\n(xacro)", p(P_IMU), textcoords="offset points",
                xytext=(8, 8), fontsize=8, color="k")
    for i, mk in [(0, "^"), (1, "v")]:
        ax.plot(*p(P_CAM_NOM[i]), "k" + mk, ms=10, mfc="none", mew=2, zorder=5)
    ax.annotate("ZED cam0/cam1\n(xacro)", p(P_CAM_NOM[0]), textcoords="offset points",
                xytext=(8, 8), fontsize=8, color="k")
    ax.plot(*p(P_DVL), "kD", ms=9, mfc="none", mew=2, zorder=5)
    ax.annotate("DVL (xacro)", p(P_DVL), textcoords="offset points",
                xytext=(8, -14), fontsize=8, color="k")
    # corpo nominal: linhas IMU->cameras e IMU->DVL
    for alvo in (P_CAM_NOM[0], P_CAM_NOM[1], P_DVL):
        ax.plot([P_IMU[ia], alvo[ia]], [P_IMU[ib], alvo[ib]], "k-", lw=1, alpha=.25, zorder=1)

    # --- calibrado, por bag ---
    for bag, d in sorted(dados.items()):
        cor = CORES.get(bag, "#555555")
        out = bag in OUTLIERS
        alpha = .35 if out else 1.0
        for i, mk in [(0, "^"), (1, "v")]:
            if i not in d["cams"]:
                continue
            lev = d["R_base_imu"] @ d["cams"][i]      # camera rel. IMU, eixos base_link
            if ancora == "imu":
                pt, base = P_IMU + lev, P_IMU
            else:                                      # ancora na camera nominal
                pt, base = P_CAM_NOM[i] - lev, P_CAM_NOM[i]
            ax.plot(*p(pt), mk, color=cor, ms=8, alpha=alpha, zorder=6,
                    label=None)
            ax.plot([base[ia], pt[ia]], [base[ib], pt[ib]], "-", color=cor,
                    lw=1, alpha=alpha * .5, zorder=2)

    ax.set_title(titulo, fontsize=10, fontweight="bold")
    ax.set_xlabel(xlab, fontsize=8)
    ax.set_ylabel(ylab, fontsize=8)
    ax.grid(True, alpha=.3, ls=":")
    ax.axhline(0, color="gray", lw=.5)
    ax.axvline(0, color="gray", lw=.5)
    ax.set_aspect("equal", adjustable="datalim")
    ax.tick_params(labelsize=7)


def figura(dados, ancora, caminho):
    fig, axes = plt.subplots(2, 2, figsize=(13, 10))
    for ax, (ia, ib, t, xl, yl) in zip(axes.flat, VISTAS):
        desenhar(ax, ia, ib, t, xl, yl, dados, ancora)

    # painel 4: legenda + numeros
    ax = axes.flat[3]
    ax.axis("off")
    handles = [plt.Line2D([], [], marker="s", ls="none", mfc="none", mec="k", mew=2,
                          ms=10, label="nominal (xacro + zed_macro)")]
    for bag in sorted(dados):
        out = bag in OUTLIERS
        handles.append(plt.Line2D([], [], marker="o", ls="none", ms=8,
                                  color=CORES.get(bag, "#555"), alpha=.35 if out else 1.0,
                                  label=bag.replace("piscina_calib_", "bag ") +
                                        (" (outlier, D9)" if out else "")))
    handles += [plt.Line2D([], [], marker="^", ls="none", mfc="none", mec="k", ms=9, label="cam0 (left)"),
                plt.Line2D([], [], marker="v", ls="none", mfc="none", mec="k", ms=9, label="cam1 (right)")]
    ax.legend(handles=handles, loc="upper left", fontsize=9, frameon=False,
              title="Legenda", title_fontsize=10)

    linhas = ["", "Posição de cam0 relativa à Microstrain (eixos base_link):", ""]
    nom = P_CAM_NOM[0] - P_IMU
    linhas.append("  nominal (xacro)   [%6.3f %6.3f %6.3f]  |.|=%.3f m"
                  % (nom[0], nom[1], nom[2], np.linalg.norm(nom)))
    vals = []
    for bag, d in sorted(dados.items()):
        lev = d["R_base_imu"] @ d["cams"][0]
        if bag not in OUTLIERS:
            vals.append(lev)
        linhas.append("  %-8s          [%6.3f %6.3f %6.3f]  |.|=%.3f m   rot %.2f°%s"
                      % (bag.replace("piscina_calib_", "bag "), lev[0], lev[1], lev[2],
                         np.linalg.norm(lev), rot_angle(d["R_base_imu"]),
                         "  (outlier)" if bag in OUTLIERS else ""))
    if len(vals) > 1:
        V = np.array(vals)
        linhas += ["", "  Sem o bag 01:  média [%6.3f %6.3f %6.3f]  desvio [%5.3f %5.3f %5.3f]"
                   % tuple(list(V.mean(axis=0)) + list(V.std(axis=0)))]
        linhas.append("  Viés vs. xacro: [%6.3f %6.3f %6.3f]  |.|=%.3f m"
                      % tuple(list(V.mean(axis=0) - nom) + [np.linalg.norm(V.mean(axis=0) - nom)]))
    ax.text(0, .45, "\n".join(linhas), fontsize=8, family="monospace",
            va="top", transform=ax.transAxes)

    sub = ("Microstrain ancorada no nominal — as câmeras se deslocam"
           if ancora == "imu" else
           "ZED ancorada no nominal — a Microstrain se desloca")
    fig.suptitle("Geometria calibrada vs. CAD  ·  %s\n(a calibração mede a posição RELATIVA: "
                 "as duas leituras são igualmente válidas)" % sub, fontsize=12, fontweight="bold")
    fig.tight_layout(rect=[0, 0, 1, .95])
    fig.savefig(caminho, dpi=140)
    print("figura escrita: %s" % caminho)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--outdir", default="/data/output")
    ap.add_argument("-o", "--out", default="/data/output/geometria-calib.png")
    a = ap.parse_args()
    dados = carregar(a.outdir)
    if not dados:
        sys.exit("nenhum *-camchain-imucam.yaml em %s" % a.outdir)
    print("bags: %s" % ", ".join(sorted(dados)))
    base, ext = os.path.splitext(a.out)
    figura(dados, "imu", base + "-ancora-imu" + ext)
    figura(dados, "cam", base + "-ancora-cam" + ext)


if __name__ == "__main__":
    main()
