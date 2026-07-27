#!/usr/bin/env python3
"""Teste do CsvDvlDatasetReader (Tarefa 3.3 do plano).

Gera medicoes sinteticas, corrompe algumas (para exercitar o gating), exporta para CSV, le de
volta com o CsvDvlDatasetReader e verifica: contagem total, contagem de validas apos gating,
round-trip de valores, e integracao com o termo de erro (n_added == numValid).

Rodar no container:
  LD_PRELOAD=/lib/x86_64-linux-gnu/libcholmod.so python3 test/dvl/test_dvl_csv_reader.py
"""
from __future__ import print_function
import os
import sys
import tempfile
import numpy as np

import aslam_splines as asp
import aslam_backend as aopt
import kalibr_common as kc

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import dvl_synthetic as ds
from kalibr_imu_camera_calibration import DvlError as de

REPO = "/catkin_ws/src/kalibr"
DVL_YAML = os.path.join(REPO, "scripts/config/dvl0.yaml")


def main():
    ok = True

    # 1. gerar medicoes (todas validas), com covariancia realista
    gt = ds.make_ground_truth_spline(seed=2)
    times = ds.sample_times(gt, n=100)
    T_gt = ds.build_T([0.1, 0.15, 0.2, 0.96], [0.15, -0.10, 0.20])
    meas = ds.generate_dvl_measurements(gt, T_gt, 1.0, times, noise_sigma=0.0,
                                        cov=(0.02 ** 2) * np.eye(3))
    n_total = len(meas)

    # 2. corromper 4 amostras, cada uma por um motivo de gating diferente
    meas[5]["velocity_valid"] = False               # sem bottom-lock
    meas[10]["altitude"] = 0.05                      # abaixo de min_altitude (0.1)
    meas[15]["fom"] = 9.9                            # acima de max_fom (1.0)
    meas[20]["velocity"] = np.array([5.0, 0.0, 0.0])  # acima de max_speed (1.5)
    n_bad = 4
    valid_ref_idx = 0                                # amostra intacta p/ round-trip

    # 3. exportar CSV
    tmp = tempfile.NamedTemporaryFile(suffix=".csv", delete=False).name
    ds.write_csv(meas, tmp)

    # 4. ler com gating do dvl0.yaml
    gating = kc.DvlParameters(DVL_YAML).getGating()
    reader = kc.CsvDvlDatasetReader(tmp, gating=gating)

    print("Total lido: %d (esperado %d)" % (reader.numMessages(), n_total))
    print("Validas apos gating: %d (esperado %d)" % (reader.numValid(), n_total - n_bad))
    if reader.numMessages() != n_total:
        ok = False; print("  FALHA: contagem total")
    if reader.numValid() != n_total - n_bad:
        ok = False; print("  FALHA: contagem de validas (gating)")

    # 5. round-trip de valores numa amostra intacta
    all_m = list(reader)
    rv = all_m[valid_ref_idx]
    if not np.allclose(rv["velocity"], meas[valid_ref_idx]["velocity"], atol=1e-6):
        ok = False; print("  FALHA: velocity nao sobreviveu ao round-trip CSV")
    if not np.allclose(rv["covariance"], meas[valid_ref_idx]["covariance"], atol=1e-9):
        ok = False; print("  FALHA: covariance nao sobreviveu ao round-trip CSV")
    if abs(rv["timestamp"] - meas[valid_ref_idx]["timestamp"]) > 1e-6:
        ok = False; print("  FALHA: timestamp nao sobreviveu ao round-trip CSV")

    # 6. integracao: os termos de erro adicionados == numValid
    problem = aopt.OptimizationProblem()
    poseSplineDv = asp.BSplinePoseDesignVariable(gt)
    for i in range(poseSplineDv.numDesignVariables()):
        dv = poseSplineDv.designVariable(i); dv.setActive(False); problem.addDesignVariable(dv)
    q_dv = aopt.RotationQuaternionDv(np.array([0., 0., 0., 1.])); q_dv.setActive(True); problem.addDesignVariable(q_dv)
    r_dv = aopt.EuclideanPointDv(np.array([0., 0., 0.])); r_dv.setActive(True); problem.addDesignVariable(r_dv)
    scale_dv = aopt.Scalar(1.0); scale_dv.setActive(False); problem.addDesignVariable(scale_dv)
    _, n_added, n_skipped = de.addDvlVelocityErrorTerms(problem, poseSplineDv, reader, q_dv, r_dv, scale_dv)
    print("Termos adicionados: %d  pulados: %d (esperado adicionados=%d)"
          % (n_added, n_skipped, reader.numValid()))
    if n_added != reader.numValid():
        ok = False; print("  FALHA: n_added != numValid")

    try:
        os.remove(tmp)
    except OSError:
        pass

    print("\nRESULTADO 3.3:", "OK - CsvDvlDatasetReader validado" if ok else "FALHOU")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
