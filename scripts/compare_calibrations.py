#!/usr/bin/env python3
"""Compara as calibracoes camera-IMU dos bags de tanque (repetibilidade cross-bag).

Tarefa 5.1 da spec .ai/specs/validacao-calibracao-tanque (R5/CA4).

Le os `*-camchain-imucam.yaml` + `*-results-imucam.txt` de cada bag e reporta:
  - T_cam_imu por bag (translacao e rotacao), com media +/- desvio entre bags;
  - as mesmas grandezas expressas nos eixos do `base_link`, comparadas com o
    nominal do xacro (`petro_rov.urdf.xacro` + `zed_macro.urdf.xacro`, zed2i);
  - erro de reprojecao e timeshift por bag.

A comparacao em `base_link` e' o que responde a duvida mecanica: a rotacao da
Microstrain deve sair ~identidade (o xacro usa rpy=0 0 0) e a posicao da ZED
relativa a Microstrain deve bater com as cotas do desenho. Se os bags concordarem
entre si mas divergirem do xacro, o desenho e' que esta' errado; se os bags
divergirem entre si, a translacao nao esta' observavel nesses datasets.

Roda no HOST (so numpy; nao precisa de ROS nem PyYAML).

Uso:
  python3 compare_calibrations.py                     # todos os bags encontrados
  python3 compare_calibrations.py --bags 01 02 03 04
  python3 compare_calibrations.py --outdir data/output
"""
from __future__ import print_function
import argparse
import glob
import os
import re
import sys

import numpy as np

# ---------------------------------------------------------------- nominal (CAD)
# petro_rov.urdf.xacro
P_ZED_LINK = np.array([0.133, 0.0, -0.015])      # base_link -> zed_node_camera_link
P_IMU = np.array([-0.09439, 0.0, -0.01112])      # base_link -> imu_link (Microstrain)
P_DVL = np.array([0.0, 0.0, -0.13992])           # base_link -> dvl_link
# zed_macro.urdf.xacro, model=zed2i
ZED_HEIGHT = 0.03
ZED_BASELINE = 0.12
ZED_OPTICAL_OFFSET_X = -0.01

P_ZED_CENTER = P_ZED_LINK + np.array([0.0, 0.0, ZED_HEIGHT / 2])
P_CAM_NOM = {
    0: P_ZED_CENTER + np.array([ZED_OPTICAL_OFFSET_X, ZED_BASELINE / 2, 0.0]),
    1: P_ZED_CENTER + np.array([ZED_OPTICAL_OFFSET_X, -ZED_BASELINE / 2, 0.0]),
}

# optical frame (x=direita, y=baixo, z=frente) -> base_link (x=frente, y=esq, z=cima)
R_BASE_OPT = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], dtype=float)


# ---------------------------------------------------------------- parsing
def parse_camchain(path):
    """Extrai {cam_idx: {T_cam_imu, timeshift}} do camchain-imucam.yaml.

    Parser proprio por regex: o formato do Kalibr e' regular e assim o script
    nao depende de PyYAML (que nao esta' instalado no host).
    """
    with open(path) as fh:
        txt = fh.read()
    out = {}
    blocks = re.split(r"^(cam\d+):$", txt, flags=re.M)[1:]
    for name, body in zip(blocks[0::2], blocks[1::2]):
        idx = int(name[3:])
        m = re.search(r"T_cam_imu:\s*((?:\s*-\s*\[[^\]]*\]\s*){4})", body)
        if not m:
            continue
        rows = [[float(v) for v in r.split(",")]
                for r in re.findall(r"-\s*\[([^\]]*)\]", m.group(1))]
        ts = re.search(r"timeshift_cam_imu:\s*([-\d.eE+]+)", body)
        out[idx] = {"T": np.array(rows), "timeshift": float(ts.group(1)) if ts else None}
    return out


def parse_results(path):
    """Erro de reprojecao (mean/median) por camera, do *-results-imucam.txt."""
    if not os.path.exists(path):
        return {}
    with open(path) as fh:
        txt = fh.read()
    out = {}
    for m in re.finditer(
            r"Reprojection error \(cam(\d+)\) \[px\]:\s*mean ([\d.eE+-]+), median ([\d.eE+-]+)", txt):
        out[int(m.group(1))] = (float(m.group(2)), float(m.group(3)))
    return out


# ---------------------------------------------------------------- geometria
def rot_angle(R):
    return np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2.0, -1.0, 1.0)))


def invert(T):
    Ti = np.eye(4)
    Ti[:3, :3] = T[:3, :3].T
    Ti[:3, 3] = -T[:3, :3].T @ T[:3, 3]
    return Ti


def spread(label, vals, unit, fmt="%8.4f"):
    """Imprime media +/- desvio (e amplitude) de uma grandeza entre bags."""
    a = np.array(vals)
    if a.ndim == 1:
        a = a[:, None]
    mean, std = a.mean(axis=0), a.std(axis=0)
    rng = a.max(axis=0) - a.min(axis=0)
    j = lambda v: " ".join(fmt % x for x in v)
    print("    %-22s media [%s]  desvio [%s]  amplitude [%s] %s"
          % (label, j(mean), j(std), j(rng), unit))


# ---------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    here = os.path.dirname(os.path.abspath(__file__))
    ap.add_argument("--outdir", default=os.path.join(here, "data", "output"),
                    help="Pasta com os resultados do Kalibr (default: %(default)s).")
    ap.add_argument("--bags", nargs="+", default=None,
                    help="Sufixos dos bags (ex.: 01 02 03 04). Default: todos os encontrados.")
    args = ap.parse_args()

    if args.bags:
        names = ["piscina_calib_%s" % b for b in args.bags]
    else:
        names = None

    # Descoberta: aceita os resultados soltos em outdir/ OU um por subdiretorio
    # (outdir/<bag>/<bag>-camchain-imucam.yaml) — este ultimo e' o layout do v2, em que
    # cada bag vive na sua pasta para que o Kalibr (que grava ao lado do bag) nao
    # sobrescreva o resultado de outro.
    paths = sorted(glob.glob(os.path.join(args.outdir, "*-camchain-imucam.yaml")) +
                   glob.glob(os.path.join(args.outdir, "*", "*-camchain-imucam.yaml")))
    if names:   # --bags filtra por nome
        paths = [p for p in paths
                 if os.path.basename(p).replace("-camchain-imucam.yaml", "") in names]
    if not paths:
        sys.exit("Nenhum *-camchain-imucam.yaml em %s (nem em subdiretorios)" % args.outdir)

    data, found = {}, []
    for p in paths:
        n = os.path.basename(p).replace("-camchain-imucam.yaml", "")
        if n in data:
            continue
        cams = parse_camchain(p)
        rep = parse_results(p.replace("-camchain-imucam.yaml", "-results-imucam.txt"))
        if 0 not in cams:
            continue
        data[n] = (cams, rep)
        found.append(n)
    if names:
        missing = [n for n in names if n not in data]
        if missing:
            print("(ainda sem resultado: %s)\n" % ", ".join(missing))

    # ---- 1) por bag, no frame da IMU (saida crua do Kalibr)
    print("=" * 100)
    print("1) T_cam_imu POR BAG  (t_ic = origem da camera no frame da Microstrain, metros)")
    print("=" * 100)
    print("  %-22s %-6s %-26s %-9s %-12s %s"
          % ("bag", "cam", "t_ic [x y z]", "|t_ic|", "timeshift", "reproj mean/median [px]"))
    for n in found:
        cams, rep = data[n]
        for i in sorted(cams):
            T_ic = invert(cams[i]["T"])
            t = T_ic[:3, 3]
            r = rep.get(i)
            print("  %-22s cam%-3d [%7.4f %7.4f %7.4f] %8.4f  %8.2f ms  %s"
                  % (n, i, t[0], t[1], t[2], np.linalg.norm(t),
                     cams[i]["timeshift"] * 1e3,
                     "%.2f / %.2f" % r if r else "-"))

    # ---- 2) nos eixos do base_link, contra o xacro
    print()
    print("=" * 100)
    print("2) COMPARACAO COM O CAD  (eixos do base_link; nominal = xacro + zed_macro/zed2i)")
    print("=" * 100)
    per_bag = {}
    for n in found:
        cams, _ = data[n]
        # R_ci de cam0 = R_opt_imu -> orientacao da Microstrain no base_link
        R_base_imu = R_BASE_OPT @ cams[0]["T"][:3, :3]
        rows = {}
        for i in sorted(cams):
            t_ic = invert(cams[i]["T"])[:3, 3]
            rows[i] = R_base_imu @ t_ic           # camera rel. Microstrain, eixos base_link
        per_bag[n] = (R_base_imu, rows)

    print("\n  a) Orientacao da Microstrain no base_link (xacro: rpy=0 0 0 -> esperado ~0 deg)")
    for n in found:
        print("     %-22s desvio da identidade: %6.2f deg" % (n, rot_angle(per_bag[n][0])))
    spread("entre bags:", [[rot_angle(per_bag[n][0])] for n in found], "deg", "%8.2f")

    print("\n  b) Posicao das cameras relativa a Microstrain")
    for i in sorted(P_CAM_NOM):
        nom = P_CAM_NOM[i] - P_IMU
        print("     cam%d  nominal (xacro) = [%7.4f %7.4f %7.4f]  |.| = %.4f m"
              % (i, nom[0], nom[1], nom[2], np.linalg.norm(nom)))
        for n in found:
            if i not in per_bag[n][1]:
                continue
            c = per_bag[n][1][i]
            d = c - nom
            print("       %-20s calib = [%7.4f %7.4f %7.4f]  dif = [%7.4f %7.4f %7.4f]  |dif| = %.4f m"
                  % (n, c[0], c[1], c[2], d[0], d[1], d[2], np.linalg.norm(d)))
        vals = [per_bag[n][1][i] for n in found if i in per_bag[n][1]]
        if len(vals) > 1:
            spread("entre bags (cam%d):" % i, vals, "m")

    # ---- 3) veredito
    print()
    print("=" * 100)
    print("3) LEITURA")
    print("=" * 100)
    if len(found) < 2:
        print("  Apenas 1 bag: sem repetibilidade. Rode os demais para concluir algo sobre o CAD.")
    else:
        vals = np.array([per_bag[n][1][0] for n in found])
        disp = float(np.linalg.norm(vals.std(axis=0)))
        difs = np.array([per_bag[n][1][0] - (P_CAM_NOM[0] - P_IMU) for n in found])
        dif_med = difs.mean(axis=0)
        vies = float(np.linalg.norm(dif_med))
        print("  dispersao entre bags (cam0, desvio-padrao 3D): %.4f m" % disp)
        print("  vies medio vs. xacro (cam0):                   %.4f m" % vies)
        print("  razao vies/dispersao:                          %.1fx" % (vies / max(disp, 1e-9)))

        # Duas perguntas independentes: (i) o CAD esta' errado? -> vies vs. dispersao;
        # (ii) com que precisao sabemos a correcao? -> dispersao.
        if vies > 3 * max(disp, 1e-9):
            print("  -> O CAD ESTA' ERRADO: o vies vs. o desenho e' %.1fx maior que a dispersao"
                  % (vies / max(disp, 1e-9)))
            print("     entre bags, ou seja, TODOS os bags concordam que a camera nao esta' onde")
            print("     o xacro diz — mesmo que discordem entre si sobre o valor exato.")
            if disp >= 0.02:
                print("     POReM a correcao so' e' conhecida com +/- %.0f cm (dispersao alta):"
                      % (disp * 100))
                print("     use o vetor abaixo como indicacao de ONDE olhar no desenho, nao como cota.")
            print("     Correcao media a aplicar no nominal (eixos base_link):")
            print("        [%+.4f %+.4f %+.4f] m" % tuple(dif_med))
            print("     Equivalente: a Microstrain estaria em base_link = [%.4f %.4f %.4f]"
                  % tuple(P_IMU - dif_med))
            print("        em vez de [%.4f %.4f %.4f] (xacro)." % tuple(P_IMU))
        elif disp >= 0.02:
            print("  -> INCONCLUSIVO: a dispersao entre bags (%.0f cm) e' da ordem do vies vs. o"
                  % (disp * 100))
            print("     xacro. A translacao camera-IMU nao esta' observavel nesses datasets;")
            print("     melhorar a excitacao rotacional antes de concluir algo sobre o CAD.")
        else:
            print("  -> Bags consistentes e proximos do xacro: CAD confere.")
        print("\n  Lever-arm Microstrain->DVL previsto pelo xacro (referencia do proximo passo):")
        print("     [%7.4f %7.4f %7.4f]  |.| = %.4f m"
              % tuple(list(P_DVL - P_IMU) + [np.linalg.norm(P_DVL - P_IMU)]))


if __name__ == "__main__":
    main()
