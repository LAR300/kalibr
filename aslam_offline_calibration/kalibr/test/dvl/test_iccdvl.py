#!/usr/bin/env python3
"""Teste do IccDvl - construcao e design variables (Tarefa 4.1 do plano).

Verifica: instanciacao a partir do config + CSV, criacao dos design variables (rotacao, lever-arm,
escala) com atividade correta, e que getResultTransformation reproduz o extrinseco do config (SE3)
antes de qualquer otimizacao (round-trip da parametrizacao interna). Tambem testa os helpers de
conversao SE3 <-> (C_dvl_b, r_b).

Rodar no container:
  LD_PRELOAD=/lib/x86_64-linux-gnu/libcholmod.so python3 test/dvl/test_iccdvl.py
"""
from __future__ import print_function
import os
import sys
import tempfile
import numpy as np

import sm
import aslam_backend as aopt
import incremental_calibration as inc
import kalibr_common as kc
from kalibr_imu_camera_calibration import IccSensors as sens

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import dvl_synthetic as ds

REPO = "/catkin_ws/src/kalibr"
DVL_YAML = os.path.join(REPO, "scripts/config/dvl0.yaml")


def main():
    ok = True

    # 0. helpers de conversao SE3 <-> params: round-trip para uma SE3 aleatoria
    C = np.array(aopt.RotationQuaternionDv(sm.r2quat(
        ds.quat_to_R([0.2, -0.3, 0.1, 0.9]))).toRotationMatrix())
    t = np.array([0.3, -0.2, 0.15])
    T = np.eye(4); T[:3, :3] = C; T[:3, 3] = t
    q, r_b = sens.dvlExtrinsicToParams(T)
    C_back = aopt.RotationQuaternionDv(q).toRotationMatrix()
    T_back = sens.dvlParamsToExtrinsic(C_back, r_b)
    if not np.allclose(T_back, T, atol=1e-9):
        ok = False; print("  FALHA: round-trip SE3 <-> (C,r_b) divergiu")
    print("Helpers SE3<->params: round-trip %s" % ("OK" if np.allclose(T_back, T, atol=1e-9) else "FALHOU"))

    # 1. gerar CSV sintetico
    gt = ds.make_ground_truth_spline(seed=2)
    times = ds.sample_times(gt, n=80)
    meas = ds.generate_dvl_measurements(gt, np.eye(4), 1.0, times, noise_sigma=0.0)
    tmp = tempfile.NamedTemporaryFile(suffix=".csv", delete=False).name
    ds.write_csv(meas, tmp)

    # 2. config com extrinseco SE3 conhecido e escala estimavel
    cfg = kc.DvlParameters(DVL_YAML)
    cfg.setExtrinsic(T)              # SE3 conhecido
    cfg.setEstimateScale(True)
    dvl = sens.IccDvl(cfg, csvfile=tmp)
    if len(dvl.dvlData) != len(meas):
        ok = False; print("  FALHA: dvlData nao carregou todas as amostras")

    # 3. design variables (problema do tipo usado no pipeline real: suporta group_id)
    problem = inc.CalibrationOptimizationProblem()
    dvl.addDesignVariables(problem)
    has_dvs = all(hasattr(dvl, a) for a in ("q_dvl_b_Dv", "r_dvl_b_Dv", "scaleDv"))
    if not has_dvs:
        ok = False; print("  FALHA: design variables nao criados")
    if not dvl.scaleDv.isActive():
        ok = False; print("  FALHA: scaleDv deveria estar ativo (estimate_scale=True)")

    # escala inativa quando estimate_scale=False
    cfg2 = kc.DvlParameters(DVL_YAML); cfg2.setEstimateScale(False)
    dvl2 = sens.IccDvl(cfg2, csvfile=tmp)
    dvl2.addDesignVariables(inc.CalibrationOptimizationProblem())
    if dvl2.scaleDv.isActive():
        ok = False; print("  FALHA: scaleDv deveria estar INATIVO (estimate_scale=False)")

    # 4. getResultTransformation reproduz o config SE3 (antes de otimizar)
    T_res = dvl.getResultTransformation()
    if not np.allclose(T_res, T, atol=1e-9):
        ok = False; print("  FALHA: getResultTransformation != config SE3\n%s\nvs\n%s" % (T_res, T))
    print("getResultTransformation reproduz o config SE3: %s"
          % ("OK" if np.allclose(T_res, T, atol=1e-9) else "FALHOU"))

    try:
        os.remove(tmp)
    except OSError:
        pass

    print("\nRESULTADO 4.1:", "OK - IccDvl (construcao + design variables) validado" if ok else "FALHOU")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
