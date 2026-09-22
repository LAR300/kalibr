#!/usr/bin/env python3
"""Inspeciona um bag ROS 2 (mcap/sqlite3/.zip) quanto a prontidao para calibracao.

Numa passada so', reporta:
  1. CONTINUIDADE   - gaps por topico (o que arruinou o dataset v1)
  2. SINCRONIA      - offset estereo left/right, por vizinho mais proximo
  3. EXCITACAO      - observabilidade do lever-arm camera-IMU
  4. INTRINSECOS    - K, D e P do camera_info (calibracao de fabrica do SDK)
  5. IMAGENS        - encoding e resolucao

Roda no HOST (lib `rosbags`; nao precisa de ROS instalado).

Uso:
  python3 inspect_bag.py <bag_dir|bag.zip|bag.mcap> [...]
  python3 inspect_bag.py --imu-topic /imu/data --left <topico> --right <topico> <bag>

## Sobre a excitacao (secao 3)

Camera e IMU sao rigidas, entao `a_cam = a_imu + R*([w']x + [w]x[w]x)*r`: o lever-arm `r` SO' e'
observavel atraves de `S = [w']x + [w]x[w]x`. Acumulando `M = sum S^T S`, os autovalores de M dao a
informacao por direcao e o autovetor do menor autovalor e' a direcao CEGA da calibracao.
`sqrt(lambda)` tem unidade (m/s^2)/m: quantos m/s^2 de sinal cada metro de lever-arm gera.
Comparando com o ruido do acelerometro sai um piso pratico de incerteza.

Reportamos a versao **centripeta** (so' `[w]x[w]x`, sem derivar o giro, imune a ruido de
diferenciacao). A versao completa exigiria derivar `w`, o que infla o resultado com ruido.
"""
from __future__ import print_function
import argparse
import sys
from pathlib import Path

import numpy as np

try:
    from rosbags.highlevel import AnyReader
except ImportError:
    sys.exit("precisa da lib rosbags:  pip install rosbags")

ACC_NOISE_DENSITY = 0.002   # m/s^2/sqrt(Hz), provisorio (ver premissas-e-fontes-de-erro.md)


def skew(v):
    return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])


def stamp_cdr(raw):
    """Le header.stamp direto dos bytes CDR, sem desserializar a mensagem.

    Layout: 4 bytes de encapsulamento, depois Header -> Time {int32 sec, uint32 nanosec}.
    Evita desserializar payloads de imagem de ~1 MB so' para ler o timestamp.

    IMPORTANTE: analises de calibracao tem de usar o **header.stamp**, nao o tempo de
    gravacao do bag. Neste dataset os dois diferem em 31 ms para a camera direita (latencia
    de serializacao) -- medir pelo tempo de bag faz o estereo parecer dessincronizado.
    """
    sec, nsec = np.frombuffer(raw, dtype="<i4", count=1, offset=4)[0], \
                np.frombuffer(raw, dtype="<u4", count=1, offset=8)[0]
    return int(sec) + int(nsec) * 1e-9


def stats_gaps(ts):
    ts = np.sort(np.asarray(ts, dtype=float))
    if len(ts) < 3:
        return None
    d = np.diff(ts)
    med = float(np.median(d))
    gaps = d[d > 2.5 * med]
    dur = ts[-1] - ts[0]
    return dict(n=len(ts), nominal=1 / med, efetivo=len(ts) / dur, dur=dur,
                ngaps=len(gaps), perdido=float(gaps.sum()), maior=float(d.max()))


def par_vizinho(a, b):
    """Offset (b - a) por vizinho mais proximo; robusto a frames perdidos."""
    a, b = np.sort(a), np.sort(b)
    if len(a) == 0 or len(b) < 2:
        return None
    i = np.clip(np.searchsorted(b, a), 1, len(b) - 1)
    cand = np.stack([b[i - 1], b[i]])
    best = cand[np.argmin(np.abs(cand - a), axis=0), np.arange(len(a))]
    return (best - a) * 1000.0


def inspecionar(path, imu_topic, left, right):
    print("\n" + "=" * 92)
    print(path)
    print("=" * 92)

    tempos, giro, info, img_meta = {}, [], {}, {}
    with AnyReader([Path(path)]) as r:
        alvo = [c for c in r.connections
                if ("image" in c.topic or "imu" in c.topic or "camera_info" in c.topic
                    or "dvl" in c.topic)]
        for conn, t, raw in r.messages(connections=alvo):
            try:
                ts_hdr = stamp_cdr(raw)          # header.stamp, nao o tempo de bag
            except Exception:
                ts_hdr = t / 1e9
            tempos.setdefault(conn.topic, []).append(ts_hdr)
            if conn.topic == imu_topic:
                m = r.deserialize(raw, conn.msgtype)
                g = m.angular_velocity
                giro.append([g.x, g.y, g.z])
            elif "camera_info" in conn.topic and conn.topic not in info:
                m = r.deserialize(raw, conn.msgtype)
                info[conn.topic] = dict(w=m.width, h=m.height, model=m.distortion_model,
                                        K=np.array(m.k), D=np.array(m.d), P=np.array(m.p))
            elif conn.topic in (left, right) and conn.topic not in img_meta:
                m = r.deserialize(raw, conn.msgtype)
                img_meta[conn.topic] = (m.encoding, m.width, m.height)

    # ---- 1. continuidade
    print("\n1) CONTINUIDADE")
    print("   %-52s %7s %9s %9s %5s %8s %8s"
          % ("topico", "n", "nominal", "efetivo", "gaps", "perdido", "maior"))
    for topic in sorted(tempos):
        s = stats_gaps(tempos[topic])
        if not s:
            continue
        alerta = "  <-- ATENCAO" if s["perdido"] > 5.0 else ""
        print("   %-52s %7d %7.1fHz %7.1fHz %5d %7.1fs %7.2fs%s"
              % (topic, s["n"], s["nominal"], s["efetivo"], s["ngaps"],
                 s["perdido"], s["maior"], alerta))

    # ---- 2. sincronia estereo
    if left in tempos and right in tempos:
        d = par_vizinho(np.array(tempos[left]), np.array(tempos[right]))
        print("\n2) SINCRONIA ESTEREO  (right - left, pareado por vizinho mais proximo)")
        print("   mediana=%.3f ms  media=%.3f ms  std=%.3f ms  |  L=%d R=%d (dif=%d)"
              % (np.median(d), d.mean(), d.std(), len(tempos[left]), len(tempos[right]),
                 len(tempos[right]) - len(tempos[left])))

    # ---- 3. excitacao
    if giro:
        w = np.array(giro)
        ts = np.sort(np.array(tempos[imu_topic]))
        dt = float(np.median(np.diff(ts)))
        n = max(1, int(round(0.05 / dt)))
        k = np.ones(n) / n
        ws = np.stack([np.convolve(w[:, i], k, mode="same") for i in range(3)], axis=1)
        M = np.zeros((3, 3))
        for wk in ws:
            W = skew(wk) @ skew(wk)
            M += W.T @ W
        lam, V = np.linalg.eigh(M / len(ws))
        s = np.sqrt(np.maximum(lam, 0))
        rms = np.sqrt((w ** 2).mean(axis=0))
        sigma_a = ACC_NOISE_DENSITY / np.sqrt(dt)
        with np.errstate(divide="ignore"):
            sig_r = sigma_a / (np.sqrt(len(ws)) * s)
        print("\n3) EXCITACAO / OBSERVABILIDADE DO LEVER-ARM   (%s)" % imu_topic)
        print("   giro RMS  = [%.4f %.4f %.4f] rad/s  =  [%.1f %.1f %.1f] deg/s"
              % tuple(list(rms) + list(np.degrees(rms))))
        print("   giro max  = [%.1f %.1f %.1f] deg/s" % tuple(np.degrees(np.abs(w).max(axis=0))))
        print("   sensibilidade centripeta (m/s^2 por metro): %s"
              % np.array2string(s[::-1], precision=4))
        print("   razao forte/fraca: %.1fx   |   direcao mais fraca (frame IMU): %s"
              % (s[-1] / max(s[0], 1e-12), np.array2string(V[:, 0], precision=3)))
        print("   piso teorico de incerteza do lever-arm: %s cm"
              % np.array2string(100 * sig_r[::-1], precision=2))
        print("   referencia: v1 = 5.9-9.2 deg/s | v2 = 3.8-7.6 deg/s | recomendado ~20 deg/s")

    # ---- 4. intrinsecos de fabrica
    if info:
        print("\n4) INTRINSECOS DE FABRICA (camera_info do SDK)")
        for topic in sorted(info):
            d = info[topic]
            K, P = d["K"], d["P"]
            print("   %s" % topic)
            print("      %dx%d  modelo=%s" % (d["w"], d["h"], d["model"]))
            print("      K (bruta)     fx=%9.3f fy=%9.3f cx=%9.3f cy=%9.3f"
                  % (K[0], K[4], K[2], K[5]))
            print("      P (retificada) fx=%9.3f fy=%9.3f cx=%9.3f cy=%9.3f   P[3]=%.4f"
                  % (P[0], P[5], P[2], P[6], P[3]))
            print("      D (%d coef) = %s" % (len(d["D"]), np.array2string(d["D"], precision=6)))
            if abs(P[0]) > 1e-6 and abs(P[3]) > 1e-9:
                print("      -> baseline implicita = -P[3]/fx = %.6f m" % (-P[3] / P[0]))

    # ---- 5. imagens
    if img_meta:
        print("\n5) IMAGENS")
        for topic in sorted(img_meta):
            enc, w_, h_ = img_meta[topic]
            print("   %-52s encoding=%-8s %dx%d" % (topic, enc, w_, h_))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bags", nargs="+")
    ap.add_argument("--imu-topic", default="/imu/data")
    ap.add_argument("--left", default="")
    ap.add_argument("--right", default="")
    a = ap.parse_args()
    for b in a.bags:
        # descobre os topicos de imagem se nao forem dados
        left, right = a.left, a.right
        if not left or not right:
            with AnyReader([Path(b)]) as r:
                cams = [c.topic for c in r.connections
                        if "image" in c.topic and "camera_info" not in c.topic]
            left = next((t for t in cams if "left" in t), left)
            right = next((t for t in cams if "right" in t), right)
        inspecionar(b, a.imu_topic, left, right)


if __name__ == "__main__":
    main()
