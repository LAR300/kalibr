#!/usr/bin/env python3
"""Saida detalhada da calibracao do DVL (Tarefa 5.2 do plano).

Roda uma recuperacao com ruido conhecido (spline fixa), verifica as estatisticas de residuo
(RMS ~ ruido, contagens) e que saveDvlResultTxt gera o arquivo com os campos esperados.

Rodar no container:
  LD_PRELOAD=/lib/x86_64-linux-gnu/libcholmod.so python3 test/dvl/test_dvl_results.py
"""
from __future__ import print_function
import os
import re
import sys
import tempfile
import numpy as np

import aslam_splines as asp
import aslam_backend as aopt
import incremental_calibration as inc
import kalibr_common as kc
from kalibr_imu_camera_calibration import IccSensors as sens
from kalibr_imu_camera_calibration.IccCalibrator import IccCalibrator

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import dvl_synthetic as ds

REPO = "/catkin_ws/src/kalibr"
DVL_YAML = os.path.join(REPO, "scripts/config/dvl0.yaml")
SIGMA = 0.02
N = 400


def main():
    ok = True

    gt = ds.make_ground_truth_spline(seed=2)
    times = ds.sample_times(gt, n=N)
    T_internal = np.eye(4); T_internal[:3, :3] = ds.quat_to_R([0.1, 0.15, 0.2, 0.96]); T_internal[:3, 3] = [0.15, -0.10, 0.20]
    meas = ds.generate_dvl_measurements(gt, T_internal, 1.1, times, noise_sigma=SIGMA,
                                        cov=(SIGMA ** 2) * np.eye(3), seed=5)
    tmp_csv = tempfile.NamedTemporaryFile(suffix=".csv", delete=False).name
    ds.write_csv(meas, tmp_csv)

    cfg = kc.DvlParameters(DVL_YAML)
    cfg.setExtrinsic(np.eye(4)); cfg.setVelocityScale(1.0); cfg.setEstimateScale(True)
    cfg.setEstimateTimeOffset(False)
    dvl = sens.IccDvl(cfg, csvfile=tmp_csv)

    problem = inc.CalibrationOptimizationProblem()
    poseSplineDv = asp.BSplinePoseDesignVariable(gt)
    for i in range(poseSplineDv.numDesignVariables()):
        d = poseSplineDv.designVariable(i); d.setActive(False); problem.addDesignVariable(d, sens.HELPER_GROUP_ID)
    dvl.addDesignVariables(problem)
    dvl.addVelocityErrorTerms(problem, poseSplineDv)

    options = aopt.Optimizer2Options()
    options.verbose = False
    options.linearSolver = aopt.BlockCholeskyLinearSystemSolver()
    options.nThreads = 2
    options.maxIterations = 60
    opt = aopt.Optimizer2(options); opt.setProblem(problem); opt.optimize()

    # estatisticas
    st = dvl.getResidualStats()
    print("n_used=%d n_total=%d n_skipped=%d rms=%.4f per_axis=%s"
          % (st["n_used"], st["n_total"], st["n_skipped"], st["rms"],
             ["%.4f" % v for v in st["per_axis_rms"]]))
    if st["n_used"] != N or st["n_skipped"] != 0:
        ok = False; print("  FALHA: contagens de residuo incorretas")
    # residuo por-eixo ~ SIGMA ; norma RMS ~ sqrt(3)*SIGMA
    if not all(0.5 * SIGMA < v < 1.5 * SIGMA for v in st["per_axis_rms"]):
        ok = False; print("  FALHA: per-axis RMS fora do esperado (~%.3f)" % SIGMA)
    if not (0.5 * np.sqrt(3) * SIGMA < st["rms"] < 1.5 * np.sqrt(3) * SIGMA):
        ok = False; print("  FALHA: RMS da norma fora do esperado (~%.3f)" % (np.sqrt(3) * SIGMA))

    # saveDvlResultTxt
    cal = IccCalibrator(); cal.registerDvl(dvl)
    tmp_txt = tempfile.NamedTemporaryFile(suffix=".txt", delete=False).name
    cal.saveDvlResultTxt(tmp_txt)
    content = open(tmp_txt).read()
    for needle in ("T_dvl_imu", "velocity_scale", "timeshift_dvl_imu", "RMS [m/s]",
                   "measurements used"):
        if needle not in content:
            ok = False; print("  FALHA: '%s' ausente no results.txt" % needle)
    m = re.search(r"RMS \[m/s\]:\s*([0-9.eE+-]+)", content)
    if m:
        rms_txt = float(m.group(1))
        if abs(rms_txt - st["rms"]) > 1e-6:
            ok = False; print("  FALHA: RMS no txt != stats")
        print("results.txt RMS=%.4f (stats=%.4f) OK" % (rms_txt, st["rms"]))
    else:
        ok = False; print("  FALHA: RMS nao encontrado no txt")

    for f in (tmp_csv, tmp_txt):
        try:
            os.remove(f)
        except OSError:
            pass

    print("\nRESULTADO 5.2:", "OK - saida detalhada do DVL validada" if ok else "FALHOU")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
