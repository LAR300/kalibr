#!/usr/bin/env python3
"""Gerador de dados sinteticos de DVL para os testes da calibracao de DVL (Fase 2 do plano).

Usa uma BSplinePose conhecida como trajetoria ground-truth e produz medicoes de velocidade
do DVL a partir de um extrinseco T_dvl_imu e uma escala s conhecidos, segundo o modelo:

    v_dvl(t) = s * C_dvl_b * ( C_b_w(t) * v_w(t) + w_b(t) x r_dvl_b )

onde v_w = linearVelocity(t) (mundo), C_b_w = orientation(t)^-1, w_b = angularVelocityBodyFrame(t),
C_dvl_b = R(q_dvl_b), r_dvl_b = translacao (lever arm) do DVL no frame do corpo (IMU de referencia).

Requer o ambiente Python do Kalibr (rodar no container com LD_PRELOAD=libcholmod.so).
"""
from __future__ import print_function
import math
import numpy as np
import sm
import bsplines


def quat_to_R(q_xyzw):
    """Matriz de rotacao a partir de quaternion [x,y,z,w] (normaliza)."""
    q = np.asarray(q_xyzw, dtype=float)
    q = q / np.linalg.norm(q)
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ])


def build_T(q_xyzw, t):
    """Transformacao 4x4 a partir de quaternion [x,y,z,w] e translacao t[3]."""
    T = np.eye(4)
    T[:3, :3] = quat_to_R(q_xyzw)
    T[:3, 3] = np.asarray(t, dtype=float)
    return T


def _analytic_curve_value(t):
    """Valor de curva [rotvec(3), pos(3)] de uma trajetoria analitica suave em t.

    Senoidais de baixa frequencia -> velocidade linear e angular fisicamente limitadas,
    porem com excitacao em todos os 6 graus de liberdade (necessario p/ observabilidade
    da rotacao, do lever-arm e da escala).
    """
    # rotacao (rad): amplitude ~0.3, freqs distintas por eixo
    rot = np.array([
        0.30 * math.sin(2 * math.pi * 0.13 * t + 0.0),
        0.25 * math.sin(2 * math.pi * 0.19 * t + 1.0),
        0.35 * math.sin(2 * math.pi * 0.11 * t + 2.0),
    ])
    # translacao (m): amplitude ~1, freqs distintas por eixo
    pos = np.array([
        1.00 * math.sin(2 * math.pi * 0.10 * t + 0.5),
        0.80 * math.sin(2 * math.pi * 0.17 * t + 1.5),
        0.50 * math.sin(2 * math.pi * 0.23 * t + 2.5),
    ])
    return np.concatenate([rot, pos])


def make_ground_truth_spline(order=6, duration=20.0, sample_rate=30.0,
                             knots_per_second=12.0, seed=0):
    """Cria uma BSplinePose ground-truth ajustando uma trajetoria analitica suave 6-DOF.

    Segue o mesmo padrao do IccSensors.initPoseSplineFromCamera: amostra poses, converte
    para valores de curva, faz unwrap do vetor de rotacao e usa initPoseSplineSparse.
    `seed` mantido por compatibilidade (a trajetoria e deterministica).
    """
    bsp = bsplines.BSplinePose(order, sm.RotationVector())
    times = np.arange(0.0, duration, 1.0 / sample_rate)
    curve = np.array([_analytic_curve_value(t) for t in times]).T  # 6 x N

    # unwrap do vetor de rotacao (evita flips de 2*pi), como no IccSensors
    for i in range(1, curve.shape[1]):
        prev = curve[3:6, i - 1]
        r = curve[3:6, i]
        angle = np.linalg.norm(r)
        if angle < 1e-12:
            continue
        axis = r / angle
        best_r, best_dist = r, np.linalg.norm(r - prev)
        for s in range(-3, 4):
            aa = axis * (angle + math.pi * 2.0 * s)
            d = np.linalg.norm(aa - prev)
            if d < best_dist:
                best_r, best_dist = aa, d
        curve[3:6, i] = best_r

    knots = int(round(duration * knots_per_second))
    bsp.initPoseSplineSparse(times, np.asmatrix(curve), knots, 1e-4)
    return bsp


def generate_dvl_measurements(spline, T_dvl_imu, scale, times,
                              noise_sigma=0.0, cov=None, seed=0):
    """Gera medicoes de velocidade do DVL nos instantes `times`.

    Args:
        spline: BSplinePose ground-truth.
        T_dvl_imu: 4x4 (DVL -> IMU/corpo). C_dvl_b = T_dvl_imu[:3,:3], r = T_dvl_imu[:3,3].
        scale: escalar s (sound-speed).
        times: iteravel de instantes (s), dentro de (t_min, t_max).
        noise_sigma: desvio padrao do ruido gaussiano por eixo (m/s). 0 = sem ruido.
        cov: 3x3 de covariancia a reportar (default: (max(noise_sigma,1e-3))^2 * I).
        seed: semente do ruido.

    Retorna lista de dicts: {timestamp, velocity(3), covariance(3x3), velocity_valid, fom, altitude}.
    """
    rng = np.random.RandomState(seed)
    C_dvl_b = np.asarray(T_dvl_imu)[:3, :3]
    r_b = np.asarray(T_dvl_imu)[:3, 3]
    if cov is None:
        s = max(noise_sigma, 1e-3)
        cov = (s * s) * np.eye(3)
    out = []
    for t in times:
        v_w = np.asarray(spline.linearVelocity(t)).flatten()
        C_b_w = np.asarray(spline.orientation(t)).T
        w_b = np.asarray(spline.angularVelocityBodyFrame(t)).flatten()
        v_true = scale * C_dvl_b.dot(C_b_w.dot(v_w) + np.cross(w_b, r_b))
        v_meas = v_true + (rng.normal(0.0, noise_sigma, 3) if noise_sigma > 0 else 0.0)
        out.append({
            "timestamp": float(t),
            "velocity": v_meas.astype(float),
            "covariance": np.asarray(cov, dtype=float),
            "velocity_valid": True,
            "fom": float(noise_sigma),
            "altitude": 2.0,
        })
    return out


def sample_times(spline, n=200, margin=0.2):
    """n instantes uniformes dentro de (t_min+margin, t_max-margin)."""
    t0 = spline.t_min() + margin
    t1 = spline.t_max() - margin
    return np.linspace(t0, t1, n)


CSV_HEADER = ("timestamp_ns,vx,vy,vz,cov00,cov01,cov02,cov10,cov11,cov12,"
              "cov20,cov21,cov22,velocity_valid,fom,altitude")


def write_csv(measurements, path):
    """Escreve as medicoes no schema de 16 colunas do CSV do DVL (ver scripts/config/dvl0_example.csv)."""
    rows = []
    for m in measurements:
        ts_ns = int(round(float(m["timestamp"]) * 1e9))
        v = np.asarray(m["velocity"], dtype=float).flatten()
        cov = np.asarray(m["covariance"], dtype=float).reshape(9)
        valid = 1 if m.get("velocity_valid", True) else 0
        row = [ts_ns, v[0], v[1], v[2]] + list(cov) + [valid, m.get("fom", 0.0), m.get("altitude", 2.0)]
        rows.append(row)
    fmt = ["%d"] + ["%.9e"] * 12 + ["%d", "%.6e", "%.6f"]
    np.savetxt(path, np.array(rows), delimiter=",", header=CSV_HEADER, comments="# ", fmt=fmt)


# --------------------------------------------------------------------------------------
# Auto-verificacao (Tarefa 2.1): com T_dvl_imu = I e s = 1, a velocidade gerada deve
# igualar C_b_w * v_w da spline (sem termo de lever arm, sem escala).
# --------------------------------------------------------------------------------------
def _self_check():
    ok = True
    bsp = make_ground_truth_spline(seed=1)
    times = sample_times(bsp, n=50)

    # Caso 0: sanidade fisica da trajetoria (v e omega limitados)
    vmax = max(np.linalg.norm(np.asarray(bsp.linearVelocity(t)).flatten()) for t in times)
    wmax = max(np.linalg.norm(np.asarray(bsp.angularVelocityBodyFrame(t)).flatten()) for t in times)
    print("Caso 0 (fisica): max|v_w|=%.3f m/s, max|w_b|=%.3f rad/s" % (vmax, wmax))
    if not (0.05 < vmax < 5.0):
        ok = False; print("  FALHA: |v_w| fora de faixa fisica")
    if not (0.05 < wmax < 5.0):
        ok = False; print("  FALHA: |w_b| fora de faixa fisica (trajetoria patologica?)")

    # Caso 1: identidade, sem escala, sem ruido
    meas = generate_dvl_measurements(bsp, np.eye(4), 1.0, times, noise_sigma=0.0)
    max_err = 0.0
    for m, t in zip(meas, times):
        v_w = np.asarray(bsp.linearVelocity(t)).flatten()
        C_b_w = np.asarray(bsp.orientation(t)).T
        expected = C_b_w.dot(v_w)
        max_err = max(max_err, np.linalg.norm(m["velocity"] - expected))
    print("Caso 1 (T=I, s=1): max||v_gerada - C_b_w*v_w|| = %.3e" % max_err)
    if max_err > 1e-9:
        ok = False; print("  FALHA: deveria ser ~0")

    # Caso 2: escala aplicada corretamente
    s = 1.3
    meas_s = generate_dvl_measurements(bsp, np.eye(4), s, times, noise_sigma=0.0)
    max_err_s = max(np.linalg.norm(ms["velocity"] - s * m["velocity"])
                    for ms, m in zip(meas_s, meas))
    print("Caso 2 (s=%.1f): max||v_s - s*v_1|| = %.3e" % (s, max_err_s))
    if max_err_s > 1e-9:
        ok = False; print("  FALHA: escala nao aplicada corretamente")

    # Caso 3: lever arm gera diferenca quando ha rotacao (w_b != 0)
    T_lever = build_T([0, 0, 0, 1], [0.2, -0.1, 0.05])
    meas_l = generate_dvl_measurements(bsp, T_lever, 1.0, times, noise_sigma=0.0)
    diff_lever = max(np.linalg.norm(ml["velocity"] - m["velocity"])
                     for ml, m in zip(meas_l, meas))
    print("Caso 3 (lever arm): max||v_lever - v_semlever|| = %.4f m/s (deve ser fisico > 0)" % diff_lever)
    # efeito ~ |w_b| * |r| ; deve ser perceptivel mas nao absurdo
    if not (1e-4 < diff_lever < 2.0):
        ok = False; print("  FALHA: efeito do lever arm fora de faixa fisica")

    # Caso 4: ruido controlado tem magnitude coerente
    sigma = 0.02
    meas_n = generate_dvl_measurements(bsp, np.eye(4), 1.0, times, noise_sigma=sigma, seed=7)
    residuals = np.array([mn["velocity"] - m["velocity"] for mn, m in zip(meas_n, meas)])
    emp = np.std(residuals)
    print("Caso 4 (ruido sigma=%.3f): std empirico = %.4f" % (sigma, emp))
    if not (0.5 * sigma < emp < 1.5 * sigma):
        ok = False; print("  FALHA: ruido fora do esperado")

    print("\nRESULTADO 2.1:", "OK - gerador sintetico validado" if ok else "FALHOU")
    return ok


if __name__ == "__main__":
    import sys
    sys.exit(0 if _self_check() else 1)
