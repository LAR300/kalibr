#!/usr/bin/env python3
"""Prior de offset temporal do DVL via correlacao cruzada (Tarefa 4.3 do plano).

Injeta um offset temporal conhecido nos timestamps do DVL (stamp = t_spline + off_gt) e verifica que
IccDvl.findTimeOffsetPrior recupera o offset (self.timeOffset ~= -off_gt) dentro de ~1 periodo de
amostragem. Tambem verifica o no-op quando estimate_time_offset=False.

Rodar no container:
  LD_PRELOAD=/lib/x86_64-linux-gnu/libcholmod.so python3 test/dvl/test_iccdvl_timeoffset.py
"""
from __future__ import print_function
import os
import sys
import tempfile
import numpy as np

import aslam_splines as asp
import kalibr_common as kc
from kalibr_imu_camera_calibration import IccSensors as sens

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import dvl_synthetic as ds

REPO = "/catkin_ws/src/kalibr"
DVL_YAML = os.path.join(REPO, "scripts/config/dvl0.yaml")


def build_dvl(off_gt, estimate=True):
    gt = ds.make_ground_truth_spline(seed=2)
    times = ds.sample_times(gt, n=400)
    T_internal = np.eye(4); T_internal[:3, 3] = [0.15, -0.10, 0.20]
    meas = ds.generate_dvl_measurements(gt, T_internal, 1.0, times, noise_sigma=0.0)
    # injeta o offset: stamp = t_spline + off_gt (a velocidade continua a de t_spline)
    for m, t in zip(meas, times):
        m["timestamp"] = float(t) + off_gt
    tmp = tempfile.NamedTemporaryFile(suffix=".csv", delete=False).name
    ds.write_csv(meas, tmp)
    cfg = kc.DvlParameters(DVL_YAML)
    cfg.setEstimateTimeOffset(estimate)
    dvl = sens.IccDvl(cfg, csvfile=tmp)
    poseSplineDv = asp.BSplinePoseDesignVariable(gt)
    dt_mean = float(np.mean(np.diff(times)))
    return dvl, poseSplineDv, tmp, dt_mean


def main():
    ok = True

    # 1. offset positivo injetado -> espera timeOffset ~ -off_gt
    off_gt = 0.30
    dvl, poseSplineDv, tmp, dt_mean = build_dvl(off_gt, estimate=True)
    est = dvl.findTimeOffsetPrior(poseSplineDv)
    expected = -off_gt
    err = abs(est - expected)
    print("off_gt=%.3f  timeOffset=%.4f  esperado=%.4f  err=%.4f s  (1 periodo=%.4f s)"
          % (off_gt, est, expected, err, dt_mean))
    if err > dt_mean:
        ok = False; print("  FALHA: offset fora de ~1 periodo de amostragem")
    os.remove(tmp)

    # 2. offset negativo
    off_gt2 = -0.20
    dvl2, psd2, tmp2, dt_mean2 = build_dvl(off_gt2, estimate=True)
    est2 = dvl2.findTimeOffsetPrior(psd2)
    err2 = abs(est2 - (-off_gt2))
    print("off_gt=%.3f  timeOffset=%.4f  esperado=%.4f  err=%.4f s" % (off_gt2, est2, -off_gt2, err2))
    if err2 > dt_mean2:
        ok = False; print("  FALHA: offset negativo fora de tolerancia")
    os.remove(tmp2)

    # 3. no-op quando desabilitado
    dvl3, psd3, tmp3, _ = build_dvl(0.30, estimate=False)
    est3 = dvl3.findTimeOffsetPrior(psd3)
    if est3 != 0.0:
        ok = False; print("  FALHA: deveria ser 0.0 quando estimate_time_offset=False")
    os.remove(tmp3)

    print("\nRESULTADO 4.3:", "OK - prior de offset temporal validado" if ok else "FALHOU")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
