#!/usr/bin/env python3
"""Teste 1 — observabilidade do lever-arm camera-IMU a partir da excitacao rotacional.

Roda DENTRO do container kalibr (precisa de `rosbag`):

  docker exec kalibr_zed bash -c '
    source /opt/ros/noetic/setup.bash
    export MPLBACKEND=Agg
    python3 /catkin_ws/src/kalibr/scripts/analyze_excitation.py'

## A física

Camera e IMU sao rigidamente acopladas, entao a aceleracao de uma difere da outra pelo
termo de corpo rigido:

    a_cam = a_imu + R_imu · ( [w']x + [w]x[w]x ) · r          (r = lever-arm no frame da IMU)

Ou seja: **r so' e' observavel atraves de S = [w']x + [w]x[w]x**. Sem rotacao, S = 0 e r
e' completamente inobservavel — a translacao camera-IMU nao aparece em lugar nenhum dos
residuos, por mais longo que seja o dataset.

Acumulando a informacao sobre r ao longo do bag:

    M = sum_k  S_k^T · S_k

Os autovalores de M dizem quanta "alavanca" cada direcao do espaco recebeu; o autovetor do
menor autovalor e' a direcao **cega** da calibracao. sqrt(lambda) tem unidade de
(m/s^2)/m = 1/s^2: e' quantos m/s^2 de sinal no acelerometro cada metro de lever-arm gera
naquela direcao. Comparando com o ruido do acelerometro sai um limite inferior pratico de
incerteza por direcao.

## Ressalva de metodo

w' vem de diferenciar o giroscopio, que amplifica ruido. Por isso reportamos DOIS
resultados:
  - **centripeto** ([w]x[w]x apenas): nao precisa derivar, imune a esse artefato;
  - **completo** (com w' suavizado): mais fiel a fisica, mas otimista se o ruido vazar.
Se os dois contarem a mesma historia, a conclusao e' robusta.
"""
from __future__ import print_function
import os
import sys

import numpy as np

try:
    import rosbag
except ImportError:
    sys.exit("precisa rodar dentro do container (source /opt/ros/noetic/setup.bash)")

# optical -> base_link, e a rotacao da Microstrain no base_link (~identidade, ver D10)
R_BASE_OPT = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], dtype=float)
SUAVIZACAO_S = 0.05          # janela do filtro antes de derivar o giro
ACC_NOISE_DENSITY = 0.002    # m/s^2/sqrt(Hz), do imu.yaml (provisorio, D3)


def skew(v):
    return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])


def ler_giro(bagpath, topic="/imu/data"):
    ts, w = [], []
    with rosbag.Bag(bagpath, "r") as bag:
        for _, msg, _ in bag.read_messages(topics=[topic]):
            ts.append(msg.header.stamp.to_sec())
            g = msg.angular_velocity
            w.append([g.x, g.y, g.z])
    return np.array(ts), np.array(w)


def media_movel(x, n):
    if n < 2:
        return x
    k = np.ones(n) / n
    return np.stack([np.convolve(x[:, i], k, mode="same") for i in range(x.shape[1])], axis=1)


def analisar(ts, w):
    dt = float(np.median(np.diff(ts)))
    n = max(1, int(round(SUAVIZACAO_S / dt)))
    w_s = media_movel(w, n)
    wdot = np.gradient(w_s, dt, axis=0)

    M_cen = np.zeros((3, 3))
    M_tot = np.zeros((3, 3))
    for wk, wdk in zip(w_s, wdot):
        Wc = skew(wk) @ skew(wk)
        M_cen += Wc.T @ Wc
        S = skew(wdk) + Wc
        M_tot += S.T @ S

    return {
        "dt": dt, "n": len(ts), "dur": ts[-1] - ts[0],
        "w_rms": np.sqrt((w ** 2).mean(axis=0)),
        "w_max": np.abs(w).max(axis=0),
        "wdot_rms": np.sqrt((wdot ** 2).mean(axis=0)),
        "M_cen": M_cen, "M_tot": M_tot,
    }


def relatar(nome, r, R_base_imu=None):
    print("\n" + "=" * 78)
    print("%s   (%d amostras, %.1f s, %.1f Hz)" % (nome, r["n"], r["dur"], 1 / r["dt"]))
    print("=" * 78)
    print("  giro RMS  [x y z] = [%.4f %.4f %.4f] rad/s   (%.1f %.1f %.1f deg/s)"
          % tuple(list(r["w_rms"]) + list(np.degrees(r["w_rms"]))))
    print("  giro max  [x y z] = [%.4f %.4f %.4f] rad/s   (%.1f %.1f %.1f deg/s)"
          % tuple(list(r["w_max"]) + list(np.degrees(r["w_max"]))))
    print("  giro' RMS [x y z] = [%.3f %.3f %.3f] rad/s^2" % tuple(r["wdot_rms"]))

    for rotulo, M in [("CENTRIPETO (sem derivar)", r["M_cen"]), ("COMPLETO (com w')", r["M_tot"])]:
        lam, V = np.linalg.eigh(M / r["n"])       # normalizado por amostra
        s = np.sqrt(np.maximum(lam, 0))           # (m/s^2) por metro de lever-arm
        print("\n  --- informacao sobre o lever-arm: %s ---" % rotulo)
        print("    sensibilidade por direcao (m/s^2 por metro): %s"
              % np.array2string(s[::-1], precision=4))
        cond = s[-1] / max(s[0], 1e-12)
        print("    razao forte/fraca: %.1fx" % cond)
        fraca = V[:, 0]
        if R_base_imu is not None:
            fraca_b = R_base_imu @ fraca
            print("    direcao MAIS FRACA (frame IMU):      %s" % np.array2string(fraca, precision=3))
            print("    direcao MAIS FRACA (eixos base_link): %s" % np.array2string(fraca_b, precision=3))
            eixos = "xyz"
            i = int(np.argmax(np.abs(fraca_b)))
            print("      -> predominantemente o eixo %s do base_link (%.0f%%)"
                  % (eixos[i], 100 * abs(fraca_b[i])))
        # incerteza teorica: sigma_r = sigma_a / (sqrt(N) * s)
        sigma_a = ACC_NOISE_DENSITY / np.sqrt(r["dt"])       # densidade -> discreto
        with np.errstate(divide="ignore"):
            sig_r = sigma_a / (np.sqrt(r["n"]) * s)
        print("    incerteza MINIMA teorica do lever-arm por direcao (cm): %s"
              % np.array2string(100 * sig_r[::-1], precision=2))
    return r


def main():
    outdir = "/data/output"
    # rotacao da Microstrain no base_link, por bag (de compare_calibrations)
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from compare_calibrations import parse_camchain

    bags = sorted(f for f in os.listdir(outdir) if f.endswith(".bag"))
    print(__doc__)
    resumo = {}
    for b in bags:
        nome = b[:-4]
        cc = os.path.join(outdir, nome + "-camchain-imucam.yaml")
        R_base_imu = None
        if os.path.exists(cc):
            cams = parse_camchain(cc)
            if 0 in cams:
                R_base_imu = R_BASE_OPT @ cams[0]["T"][:3, :3]
        ts, w = ler_giro(os.path.join(outdir, b))
        if len(ts) == 0:
            print("%s: sem /imu/data" % nome)
            continue
        resumo[nome] = relatar(nome, analisar(ts, w), R_base_imu)

    print("\n" + "=" * 78)
    print("COMPARACAO ENTRE BAGS (giro RMS, rad/s)")
    print("=" * 78)
    for nome, r in sorted(resumo.items()):
        print("  %-22s [%.4f %.4f %.4f]  |w|=%.4f" %
              (nome, r["w_rms"][0], r["w_rms"][1], r["w_rms"][2],
               np.linalg.norm(r["w_rms"])))


if __name__ == "__main__":
    main()
