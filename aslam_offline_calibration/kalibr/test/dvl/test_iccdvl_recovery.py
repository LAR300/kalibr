#!/usr/bin/env python3
"""Recuperacao ponta a ponta via IccDvl (Tarefa 4.2 do plano).

Gera medicoes com extrinseco/escala conhecidos, monta um IccDvl (chute inicial identidade/1.0),
adiciona os design variables e os residuos de velocidade (IccDvl.addVelocityErrorTerms) contra uma
spline ground-truth FIXA, otimiza e verifica a recuperacao. Valida o caminho completo do IccDvl.

Rodar no container:
  LD_PRELOAD=/lib/x86_64-linux-gnu/libcholmod.so python3 test/dvl/test_iccdvl_recovery.py
"""
from __future__ import print_function
import os
import sys
import tempfile
import numpy as np

import aslam_splines as asp
import aslam_backend as aopt
import incremental_calibration as inc
import kalibr_common as kc
from kalibr_imu_camera_calibration import IccSensors as sens

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import dvl_synthetic as ds

REPO = "/catkin_ws/src/kalibr"
DVL_YAML = os.path.join(REPO, "scripts/config/dvl0.yaml")

C_GT = ds.quat_to_R([0.10, 0.15, 0.20, 0.96])   # rotacao C_dvl_b ground-truth
R_GT = np.array([0.15, -0.10, 0.20])            # lever arm r_b ground-truth
S_GT = 1.15                                      # escala ground-truth


def rotation_angle_deg(R_est, R_gt):
    c = (np.trace(R_est.T.dot(R_gt)) - 1.0) / 2.0
    return np.degrees(np.arccos(max(-1.0, min(1.0, c))))


def main():
    ok = True

    # 1. gerar medicoes (packing interno [C_dvl_b | r_b])
    gt = ds.make_ground_truth_spline(seed=2)
    times = ds.sample_times(gt, n=400)
    T_internal = np.eye(4); T_internal[:3, :3] = C_GT; T_internal[:3, 3] = R_GT
    meas = ds.generate_dvl_measurements(gt, T_internal, S_GT, times,
                                        noise_sigma=0.0, cov=(0.02 ** 2) * np.eye(3))
    tmp = tempfile.NamedTemporaryFile(suffix=".csv", delete=False).name
    ds.write_csv(meas, tmp)

    # 2. IccDvl com chute inicial identidade / escala 1.0
    cfg = kc.DvlParameters(DVL_YAML)
    cfg.setExtrinsic(np.eye(4))
    cfg.setVelocityScale(1.0)
    cfg.setEstimateScale(True)
    cfg.setEstimateTimeOffset(False)
    dvl = sens.IccDvl(cfg, csvfile=tmp)

    # 3. problema: spline ground-truth FIXA + DVs do DVL
    problem = inc.CalibrationOptimizationProblem()
    poseSplineDv = asp.BSplinePoseDesignVariable(gt)
    for i in range(poseSplineDv.numDesignVariables()):
        dv = poseSplineDv.designVariable(i); dv.setActive(False)
        problem.addDesignVariable(dv, sens.HELPER_GROUP_ID)
    dvl.addDesignVariables(problem)
    dvl.addVelocityErrorTerms(problem, poseSplineDv)

    # 4. otimizar
    options = aopt.Optimizer2Options()
    options.verbose = False
    options.linearSolver = aopt.BlockCholeskyLinearSystemSolver()
    options.nThreads = 2
    options.convergenceDeltaX = 1e-8
    options.convergenceDeltaJ = 1e-6
    options.maxIterations = 60
    optimizer = aopt.Optimizer2(options)
    optimizer.setProblem(problem)
    optimizer.optimize()

    # 5. checar recuperacao (parametros internos)
    C_est = dvl.q_dvl_b_Dv.toRotationMatrix()
    r_est = np.asarray(dvl.r_dvl_b_Dv.toEuclidean()).flatten()
    s_est = dvl.getResultScale()
    rot_err = rotation_angle_deg(C_est, C_GT)
    trans_err = np.linalg.norm(r_est - R_GT)
    scale_err = abs(s_est - S_GT)
    print("rot_err=%.4f deg  trans_err=%.5f m  s_est=%.5f (gt=%.3f) scale_err=%.2e"
          % (rot_err, trans_err, s_est, S_GT, scale_err))
    if rot_err > 0.1 or trans_err > 1e-3 or scale_err > 1e-3:
        ok = False; print("  FALHA: recuperacao fora da tolerancia")

    # 6. getResultTransformation coerente com os params recuperados (SE3)
    T_res = dvl.getResultTransformation()
    T_expect = sens.dvlParamsToExtrinsic(C_est, r_est)
    if not np.allclose(T_res, T_expect, atol=1e-9):
        ok = False; print("  FALHA: getResultTransformation inconsistente")

    try:
        os.remove(tmp)
    except OSError:
        pass

    print("\nRESULTADO 4.2:", "OK - recuperacao via IccDvl validada" if ok else "FALHOU")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
