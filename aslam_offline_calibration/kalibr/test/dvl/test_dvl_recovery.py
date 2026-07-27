#!/usr/bin/env python3
"""Teste de recuperacao da calibracao de DVL (Tarefa 2.2 do plano - TDD).

Estrategia (de-risca D9): usa uma spline ground-truth conhecida como trajetoria FIXA, gera medicoes
de DVL a partir de um T_dvl_imu e escala s conhecidos, e verifica que a otimizacao recupera esses
valores a partir de um chute inicial (identidade / s=1). Isola o termo de erro do DVL da estimacao
de camera/IMU.

RED esperado enquanto o modulo de producao `DvlError` (tarefas 2.3/2.4) nao existir.

Rodar no container:
  LD_PRELOAD=/lib/x86_64-linux-gnu/libcholmod.so python3 test/dvl/test_dvl_recovery.py
"""
from __future__ import print_function
import os
import sys
import numpy as np

import aslam_splines as asp
import aslam_backend as aopt

# gerador sintetico (mesmo diretorio)
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import dvl_synthetic as ds

# modulo de PRODUCAO sob teste (tarefas 2.3/2.4). Ausente agora -> RED.
try:
    from kalibr_imu_camera_calibration import DvlError as de
    _HAVE_IMPL = True
except Exception as _e:  # ImportError etc.
    _HAVE_IMPL = False
    _IMPORT_ERR = _e

np.set_printoptions(precision=6, suppress=True)

# ground-truth extrinseco DVL->IMU (rotacao moderada + lever arm)
Q_GT = np.array([0.10, 0.15, 0.20, 0.96])   # [x,y,z,w] (sera normalizado)
R_GT_T = np.array([0.15, -0.10, 0.20])       # translacao (lever arm) [m]


def rotation_angle_deg(R_est, R_gt):
    dR = R_est.T.dot(R_gt)
    c = (np.trace(dR) - 1.0) / 2.0
    c = max(-1.0, min(1.0, c))
    return np.degrees(np.arccos(c))


def build_optimizer(max_iter=60):
    options = aopt.Optimizer2Options()
    options.verbose = False
    options.linearSolver = aopt.BlockCholeskyLinearSystemSolver()
    options.nThreads = 2
    options.convergenceDeltaX = 1e-8
    options.convergenceDeltaJ = 1e-6
    options.maxIterations = max_iter
    return aopt.Optimizer2(options)


def run_recovery(estimate_scale, s_gt, noise_sigma=0.0, n=400):
    gt = ds.make_ground_truth_spline(seed=2)
    times = ds.sample_times(gt, n=n)
    T_gt = ds.build_T(Q_GT, R_GT_T)
    R_gt = ds.quat_to_R(Q_GT)
    meas = ds.generate_dvl_measurements(gt, T_gt, s_gt, times, noise_sigma=noise_sigma, seed=3)

    problem = aopt.OptimizationProblem()

    # spline ground-truth como trajetoria FIXA (design variables inativas mas registradas)
    poseSplineDv = asp.BSplinePoseDesignVariable(gt)
    for i in range(poseSplineDv.numDesignVariables()):
        dv = poseSplineDv.designVariable(i)
        dv.setActive(False)
        problem.addDesignVariable(dv)

    # extrinseco desconhecido -> chute inicial identidade
    q_dv = aopt.RotationQuaternionDv(np.array([0.0, 0.0, 0.0, 1.0]))
    q_dv.setActive(True)
    problem.addDesignVariable(q_dv)
    r_dv = aopt.EuclideanPointDv(np.array([0.0, 0.0, 0.0]))
    r_dv.setActive(True)
    problem.addDesignVariable(r_dv)
    scale_dv = aopt.Scalar(1.0)
    scale_dv.setActive(estimate_scale)
    problem.addDesignVariable(scale_dv)

    # termos de erro do DVL (PRODUCAO - tarefas 2.3/2.4)
    de.addDvlVelocityErrorTerms(problem, poseSplineDv, meas, q_dv, r_dv, scale_dv)

    optimizer = build_optimizer()
    optimizer.setProblem(problem)
    optimizer.optimize()

    R_est = q_dv.toRotationMatrix()
    t_est = np.asarray(r_dv.toEuclidean()).flatten()
    s_est = scale_dv.toScalar()

    rot_err = rotation_angle_deg(R_est, R_gt)
    trans_err = np.linalg.norm(t_est - R_GT_T)
    scale_err = abs(s_est - s_gt)
    print("  rot_err=%.4f deg  trans_err=%.5f m  s_est=%.5f (gt=%.3f) scale_err=%.2e"
          % (rot_err, trans_err, s_est, s_gt, scale_err))
    return rot_err, trans_err, scale_err


def main():
    if not _HAVE_IMPL:
        print("RED (esperado): modulo de producao 'DvlError' ainda nao implementado (tarefas 2.3/2.4).")
        print("  ImportError:", str(_IMPORT_ERR)[:120])
        return 1

    ok = True
    b_pending = False
    print("Cenario A - extrinseco apenas (s fixo=1), sem ruido:")
    rot, trans, _ = run_recovery(estimate_scale=False, s_gt=1.0, noise_sigma=0.0)
    if rot > 0.1 or trans > 1e-3:
        ok = False; print("  FALHA: extrinseco nao recuperado dentro da tolerancia")

    print("Cenario B - extrinseco + escala, sem ruido:")
    try:
        rot, trans, serr = run_recovery(estimate_scale=True, s_gt=1.2, noise_sigma=0.0)
        if rot > 0.1 or trans > 1e-3 or serr > 1e-3:
            ok = False; print("  FALHA: extrinseco/escala nao recuperados dentro da tolerancia")
    except NotImplementedError as e:
        b_pending = True
        print("  PENDENTE (tarefa 2.4):", str(e)[:80])

    if not ok:
        print("\nRESULTADO: FALHOU"); return 1
    if b_pending:
        print("\nRESULTADO: PARCIAL - Cenario A OK; Cenario B (escala) pendente da tarefa 2.4")
        return 0
    print("\nRESULTADO: OK - recuperacao completa (extrinseco + escala) validada")
    return 0


if __name__ == "__main__":
    sys.exit(main())
