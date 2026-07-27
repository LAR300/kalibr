#!/usr/bin/env python3
"""Extensoes de DVL no IccCalibrator (Tarefa 4.4 do plano).

Verifica o que e testavel sem o pipeline de camera (que exige imagens reais do AprilGrid):
  - registerDvl / DvlList
  - saveDvlParametersYaml (escreve T_dvl_imu SE3, escala, timeshift; reread confere)
  - fixCamImuDesignVariables (Modo A): desativa os DVs de calibracao camera-IMU sobre objetos reais

O buildProblem ponta a ponta (com imagens reais) e verificado no smoke test da CLI (Fase 5) e nos
dados reais do tanque (Fase 6).

Rodar no container:
  LD_PRELOAD=/lib/x86_64-linux-gnu/libcholmod.so python3 test/dvl/test_icccalibrator_dvl.py
"""
from __future__ import print_function
import os
import sys
import tempfile
import numpy as np
import yaml

import sm
import aslam_backend as aopt
import incremental_calibration as inc
import kalibr_common as kc
from kalibr_imu_camera_calibration import IccSensors as sens
from kalibr_imu_camera_calibration.IccCalibrator import IccCalibrator

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import dvl_synthetic as ds

REPO = "/catkin_ws/src/kalibr"
DVL_YAML = os.path.join(REPO, "scripts/config/dvl0.yaml")


# --- harness minimo com design variables reais (sem o pipeline de camera) ---
class _FakeCam(object):
    def __init__(self):
        self.T_c_b_Dv = aopt.TransformationDv(sm.Transformation(),
                                              rotationActive=True, translationActive=True)
        self.cameraTimeToImuTimeDv = aopt.Scalar(0.0)
        self.cameraTimeToImuTimeDv.setActive(True)


class _FakeChain(object):
    def __init__(self):
        self.camList = [_FakeCam()]


class _FakeImu(object):
    def __init__(self):
        self.q_i_b_Dv = aopt.RotationQuaternionDv(np.array([0., 0., 0., 1.]))
        self.q_i_b_Dv.setActive(True)
        self.r_b_Dv = aopt.EuclideanPointDv(np.array([0., 0., 0.]))
        self.r_b_Dv.setActive(True)


def main():
    ok = True

    # dados sinteticos do DVL
    gt = ds.make_ground_truth_spline(seed=2)
    times = ds.sample_times(gt, n=50)
    meas = ds.generate_dvl_measurements(gt, np.eye(4), 1.0, times, noise_sigma=0.0)
    tmp_csv = tempfile.NamedTemporaryFile(suffix=".csv", delete=False).name
    ds.write_csv(meas, tmp_csv)

    # 1. registerDvl
    cal = IccCalibrator()
    if cal.DvlList != []:
        ok = False; print("  FALHA: DvlList deveria iniciar vazio")
    T_known = ds.build_T([0.10, 0.15, 0.20, 0.96], [0.15, -0.10, 0.20])  # SE3 conhecido
    cfg = kc.DvlParameters(DVL_YAML)
    cfg.setExtrinsic(T_known)
    cfg.setVelocityScale(1.25)
    cfg.setEstimateScale(True)
    dvl = sens.IccDvl(cfg, csvfile=tmp_csv)
    cal.registerDvl(dvl)
    if len(cal.DvlList) != 1:
        ok = False; print("  FALHA: registerDvl nao registrou")

    # 2. saveDvlParametersYaml (precisa dos DVs criados)
    dvl.addDesignVariables(inc.CalibrationOptimizationProblem())
    out = tempfile.NamedTemporaryFile(suffix=".yaml", delete=False).name
    cal.saveDvlParametersYaml(out)
    res = yaml.safe_load(open(out))
    T_out = np.array(res["dvl0"]["T_dvl_imu"])
    if not np.allclose(T_out, T_known, atol=1e-9):
        ok = False; print("  FALHA: T_dvl_imu salvo != conhecido\n%s" % T_out)
    if abs(res["dvl0"]["velocity_scale"] - 1.25) > 1e-9:
        ok = False; print("  FALHA: velocity_scale salvo incorreto")
    if abs(res["dvl0"]["timeshift_dvl_imu"] - 0.0) > 1e-12:
        ok = False; print("  FALHA: timeshift salvo incorreto")
    print("saveDvlParametersYaml: T_dvl_imu/escala/timeshift OK")

    # 3. fixCamImuDesignVariables (Modo A) sobre DVs reais
    cal.CameraChain = _FakeChain()
    cal.ImuList = [_FakeImu()]
    cam = cal.CameraChain.camList[0]
    imu = cal.ImuList[0]
    before = (cam.T_c_b_Dv.getDesignVariable(0).isActive(),
              cam.cameraTimeToImuTimeDv.isActive(),
              imu.q_i_b_Dv.isActive(), imu.r_b_Dv.isActive())
    cal.fixCamImuDesignVariables()
    after = [cam.T_c_b_Dv.getDesignVariable(i).isActive()
             for i in range(cam.T_c_b_Dv.numDesignVariables())]
    after += [cam.cameraTimeToImuTimeDv.isActive(), imu.q_i_b_Dv.isActive(), imu.r_b_Dv.isActive()]
    print("Modo A: antes(ativos)=%s -> depois(algum ativo)=%s" % (all(before), any(after)))
    if not all(before):
        ok = False; print("  FALHA: DVs cam-IMU deveriam iniciar ativos")
    if any(after):
        ok = False; print("  FALHA: fixCamImuDesignVariables nao desativou todos os DVs cam-IMU")

    for f in (tmp_csv, out):
        try:
            os.remove(f)
        except OSError:
            pass

    print("\nRESULTADO 4.4:", "OK - extensoes de DVL no IccCalibrator validadas" if ok else "FALHOU")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
