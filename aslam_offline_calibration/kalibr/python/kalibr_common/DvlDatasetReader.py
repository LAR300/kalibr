"""Leitor de dataset de DVL a partir de CSV para o kalibr_calibrate_dvl.

O CSV (16 colunas, ver scripts/config/dvl0_example.csv) e extraido do bag ROS 2 (msg dvl_msgs/DVL).
O leitor aplica o gating (D7): descarta amostras sem bottom-lock / fora de faixa, marcando
`velocity_valid=False` (o termo de erro do DVL pula essas). Ver .ai/specs/dvl-calibration/decisions.md.
"""
import numpy as np

# ordem das colunas do CSV (ver scripts/config/dvl0_example.csv)
_N_COLS = 16
_C_TS = 0
_C_VEL = slice(1, 4)
_C_COV = slice(4, 13)
_C_VALID = 13
_C_FOM = 14
_C_ALT = 15


class CsvDvlDatasetReader(object):
    def __init__(self, csvfile, gating=None, bag_from_to=None):
        """Args:
            csvfile: caminho do CSV do DVL.
            gating: dict de limiares (ver DvlParameters.getGating()); None = sem gating.
            bag_from_to: (t0, t1) em segundos relativos ao inicio, para truncar o intervalo.
        """
        self.csvfile = csvfile
        self.gating = gating or {}
        data = np.loadtxt(csvfile, delimiter=",", comments="#", ndmin=2)
        if data.size == 0:
            raise RuntimeError("CSV do DVL vazio: {0}".format(csvfile))
        if data.shape[1] != _N_COLS:
            raise RuntimeError("CSV do DVL deve ter {0} colunas, tem {1}: {2}".format(
                _N_COLS, data.shape[1], csvfile))
        # ordena por timestamp
        data = data[np.argsort(data[:, _C_TS])]

        if bag_from_to is not None:
            t0 = data[0, _C_TS] * 1e-9
            secs = data[:, _C_TS] * 1e-9 - t0
            keep = (secs >= bag_from_to[0]) & (secs <= bag_from_to[1])
            data = data[keep]

        self.measurements = [self._row_to_measurement(r) for r in data]

    def _passes_gating(self, raw_valid, fom, altitude, speed):
        g = self.gating
        if g.get("require_velocity_valid", True) and raw_valid < 0.5:
            return False
        if altitude < g.get("min_altitude", 0.0):
            return False
        if altitude > g.get("max_altitude", float("inf")):
            return False
        if fom > g.get("max_fom", float("inf")):
            return False
        if speed > g.get("max_speed", float("inf")):
            return False
        return True

    def _row_to_measurement(self, r):
        ts = float(r[_C_TS]) * 1e-9                      # ns -> s
        velocity = np.array(r[_C_VEL], dtype=float)
        covariance = np.array(r[_C_COV], dtype=float).reshape(3, 3)
        raw_valid = float(r[_C_VALID])
        fom = float(r[_C_FOM])
        altitude = float(r[_C_ALT])
        valid = self._passes_gating(raw_valid, fom, altitude, float(np.linalg.norm(velocity)))
        # dict compativel com DvlError.addDvlVelocityErrorTerms
        return {
            "timestamp": ts,
            "velocity": velocity,
            "covariance": covariance,
            "velocity_valid": bool(valid),
            "fom": fom,
            "altitude": altitude,
        }

    def __iter__(self):
        return iter(self.measurements)

    def __len__(self):
        return len(self.measurements)

    def numMessages(self):
        return len(self.measurements)

    def numValid(self):
        return sum(1 for m in self.measurements if m["velocity_valid"])

    def validMeasurements(self):
        return [m for m in self.measurements if m["velocity_valid"]]
