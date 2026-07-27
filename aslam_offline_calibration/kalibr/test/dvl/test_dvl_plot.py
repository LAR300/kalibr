#!/usr/bin/env python3
"""Grafico de residuos do DVL (Tarefa 5.3 do plano).

Gera os residuos (recuperacao com spline fixa), plota via IccPlots.plotDvlVelocityError e salva num
PDF headless (backend Agg), verificando que o arquivo e criado sem erro.

Rodar no container:
  LD_PRELOAD=/lib/x86_64-linux-gnu/libcholmod.so python3 test/dvl/test_dvl_plot.py
"""
from __future__ import print_function
import matplotlib
matplotlib.use("Agg")  # headless ANTES de qualquer import de pylab

import os
import sys
import tempfile
import numpy as np
import pylab as pl

import aslam_splines as asp
import aslam_backend as aopt
import incremental_calibration as inc
import kalibr_common as kc
from kalibr_imu_camera_calibration import IccSensors as sens
from kalibr_imu_camera_calibration import IccPlots as plots
from kalibr_imu_camera_calibration.IccCalibrator import IccCalibrator

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import dvl_synthetic as ds

REPO = "/catkin_ws/src/kalibr"
DVL_YAML = os.path.join(REPO, "scripts/config/dvl0.yaml")


def main():
    ok = True

    gt = ds.make_ground_truth_spline(seed=2)
    times = ds.sample_times(gt, n=200)
    T_internal = np.eye(4); T_internal[:3, 3] = [0.15, -0.10, 0.20]
    meas = ds.generate_dvl_measurements(gt, T_internal, 1.1, times, noise_sigma=0.02,
                                        cov=(0.02 ** 2) * np.eye(3), seed=5)
    tmp_csv = tempfile.NamedTemporaryFile(suffix=".csv", delete=False).name
    ds.write_csv(meas, tmp_csv)

    cfg = kc.DvlParameters(DVL_YAML)
    cfg.setExtrinsic(np.eye(4)); cfg.setEstimateScale(True); cfg.setEstimateTimeOffset(False)
    dvl = sens.IccDvl(cfg, csvfile=tmp_csv)

    problem = inc.CalibrationOptimizationProblem()
    poseSplineDv = asp.BSplinePoseDesignVariable(gt)
    for i in range(poseSplineDv.numDesignVariables()):
        d = poseSplineDv.designVariable(i); d.setActive(False)
        problem.addDesignVariable(d, sens.HELPER_GROUP_ID)
    dvl.addDesignVariables(problem)
    dvl.addVelocityErrorTerms(problem, poseSplineDv)

    options = aopt.Optimizer2Options()
    options.verbose = False
    options.linearSolver = aopt.BlockCholeskyLinearSystemSolver()
    options.maxIterations = 40
    opt = aopt.Optimizer2(options); opt.setProblem(problem); opt.optimize()

    cal = IccCalibrator(); cal.registerDvl(dvl)

    # plota e salva
    tmp_pdf = tempfile.NamedTemporaryFile(suffix=".pdf", delete=False).name
    try:
        f = pl.figure(1)
        plots.plotDvlVelocityError(cal, 0, fno=f.number, noShow=True)
        f.savefig(tmp_pdf)
        pl.close(f)
    except Exception as e:
        ok = False
        print("  FALHA ao gerar o grafico:", type(e).__name__, str(e)[:120])

    if os.path.exists(tmp_pdf) and os.path.getsize(tmp_pdf) > 1000:
        print("PDF do grafico gerado: %d bytes" % os.path.getsize(tmp_pdf))
    else:
        ok = False; print("  FALHA: PDF nao gerado ou vazio")

    for fpath in (tmp_csv, tmp_pdf):
        try:
            os.remove(fpath)
        except OSError:
            pass

    print("\nRESULTADO 5.3:", "OK - grafico de residuos do DVL validado" if ok else "FALHOU")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
