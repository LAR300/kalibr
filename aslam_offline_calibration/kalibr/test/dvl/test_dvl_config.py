#!/usr/bin/env python3
"""Teste de round-trip do DvlParameters (Tarefa 3.2 do plano).

Verifica que ler -> escrever -> reler o YAML de config do DVL e idempotente e que os getters
retornam os valores esperados. Tambem exercita o caminho createYaml=True (construir via setters).

Rodar no container:
  LD_PRELOAD=/lib/x86_64-linux-gnu/libcholmod.so python3 test/dvl/test_dvl_config.py
"""
from __future__ import print_function
import os
import sys
import tempfile
import numpy as np

import kalibr_common as kc

REPO = "/catkin_ws/src/kalibr"
EXAMPLE = os.path.join(REPO, "scripts/config/dvl0.yaml")


def main():
    ok = True

    # 1. ler o exemplo
    dvl = kc.DvlParameters(EXAMPLE)
    T0 = dvl.getExtrinsic()
    ss0 = dvl.getSoundSpeed()
    g0 = dvl.getGating()
    print("Lido dvl0.yaml: sound_speed=%s estimate_scale=%s T shape=%s"
          % (ss0, dvl.getEstimateScale(), T0.shape))

    # 2. escrever em arquivo temporario e reler (round-trip)
    tmp = tempfile.NamedTemporaryFile(suffix=".yaml", delete=False).name
    dvl.writeYaml(tmp)
    dvl2 = kc.DvlParameters(tmp)

    if not np.allclose(dvl2.getExtrinsic(), T0):
        ok = False; print("  FALHA: T_dvl_imu nao sobreviveu ao round-trip")
    if dvl2.getSoundSpeed() != ss0:
        ok = False; print("  FALHA: sound_speed mudou no round-trip")
    if dvl2.getGating() != g0:
        ok = False; print("  FALHA: gating mudou no round-trip")
    if dvl2.getEstimateScale() != dvl.getEstimateScale():
        ok = False; print("  FALHA: estimate_scale mudou no round-trip")
    print("Round-trip (ler->escrever->reler): %s" % ("idempotente" if ok else "DIVERGIU"))

    # 3. construir do zero via setters (createYaml=True) e reler
    tmp2 = tempfile.NamedTemporaryFile(suffix=".yaml", delete=False).name
    fresh = kc.DvlParameters(tmp2, createYaml=True)
    fresh.setCsvPath("/data/x/dvl0.csv")
    fresh.setUpdateRate(8)
    fresh.setSoundSpeed(1480.0)
    fresh.setVelocityScale(1.0)
    fresh.setEstimateScale(True)
    fresh.setEstimateTimeOffset(False)
    T_set = np.eye(4); T_set[:3, 3] = [0.2, -0.1, 0.05]
    fresh.setExtrinsic(T_set)
    fresh.setVelocityNoiseDensity(0.03)
    fresh.setGating({"require_velocity_valid": True, "min_altitude": 0.2,
                     "max_altitude": 40.0, "max_fom": 0.5, "max_speed": 1.2})
    fresh.writeYaml()

    back = kc.DvlParameters(tmp2)
    checks = [
        ("csv", back.getCsvPath() == "/data/x/dvl0.csv"),
        ("update_rate", back.getUpdateRate() == 8),
        ("sound_speed", back.getSoundSpeed() == 1480.0),
        ("estimate_time_offset", back.getEstimateTimeOffset() is False),
        ("T_dvl_imu", np.allclose(back.getExtrinsic(), T_set)),
        ("noise_density", back.getVelocityNoiseDensity() == 0.03),
        ("gating.max_fom", back.getGating()["max_fom"] == 0.5),
    ]
    for name, good in checks:
        if not good:
            ok = False; print("  FALHA (createYaml): %s" % name)
    print("Construcao via setters (createYaml=True): %s" % ("OK" if all(c[1] for c in checks) else "FALHOU"))

    # limpeza
    for f in (tmp, tmp2):
        try:
            os.remove(f)
        except OSError:
            pass

    print("\nRESULTADO 3.2:", "OK - DvlParameters round-trip validado" if ok else "FALHOU")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
