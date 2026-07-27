"""Termos de erro de velocidade do DVL para a calibracao offline no Kalibr.

Modelo de medicao (ver .ai/specs/dvl-calibration/decisions.md D3/D4):

    v_dvl(t) = s * C_dvl_b * ( C_b_w(t) * v_w(t) + w_b(t) x r_dvl_b )

- v_w = poseSplineDv.linearVelocity(t)          (velocidade linear no frame mundo)
- C_b_w = poseSplineDv.orientation(t)^-1
- w_b  = poseSplineDv.angularVelocityBodyFrame(t)
- C_dvl_b, r_dvl_b = extrinseco DVL -> IMU de referencia (design variables)
- s = escala (sound-speed), design variable escalar

A GEOMETRIA e montada em Python puro reusando expressoes do aslam (validado no spike 1.1).
A ESCALA, quando estimada, usa o error term C++ `ket.DvlVelocityError` (tarefa 2.4), pois a
multiplicacao EuclideanExpression x ScalarExpression nao e exposta em Python (spike 1.2).
"""
import numpy as np
import aslam_backend as aopt
import kalibr_errorterms as ket


def invR_from_covariance(cov, fallback_sigma=None):
    """Inversa da covariancia 3x3 do DVL (whitening).

    Fallback quando a covariancia e invalida/singular/ausente: usa `fallback_sigma` (cov = sigma^2 I)
    se fornecido; caso contrario, identidade.
    """
    def _fallback():
        if fallback_sigma is not None and fallback_sigma > 0.0:
            return np.eye(3) / (fallback_sigma * fallback_sigma)
        return np.eye(3)

    cov = np.asarray(cov, dtype=float)
    if cov.shape != (3, 3) or not np.all(np.isfinite(cov)):
        return _fallback()
    # cov ~ 0 (sem covariancia reportada) ou singular -> fallback
    if np.trace(cov) <= 0.0 or np.linalg.matrix_rank(cov) < 3:
        return _fallback()
    try:
        return np.linalg.inv(cov)
    except np.linalg.LinAlgError:
        return _fallback()


def predicted_velocity_expression(poseSplineDv, tk, q_dvl_b_Dv, r_dvl_b_Dv):
    """EuclideanExpression da velocidade do DVL (sem escala) no instante tk.

        C_dvl_b * ( C_b_w * v_w + w_b x r_b )
    """
    v_w = poseSplineDv.linearVelocity(tk)                 # EuclideanExpression (mundo)
    C_b_w = poseSplineDv.orientation(tk).inverse()        # RotationExpression
    w_b = poseSplineDv.angularVelocityBodyFrame(tk)       # EuclideanExpression (corpo)
    C_dvl_b = q_dvl_b_Dv.toExpression()                   # RotationExpression
    r_b = r_dvl_b_Dv.toExpression()                       # EuclideanExpression
    return C_dvl_b * (C_b_w * v_w + w_b.cross(r_b))


def addDvlVelocityErrorTerms(problem, poseSplineDv, measurements,
                             q_dvl_b_Dv, r_dvl_b_Dv, scale_dv,
                             time_offset=0.0, huber=0.0, fallback_sigma=None):
    """Adiciona um termo de erro de velocidade do DVL por medicao valida.

    Args:
        problem: OptimizationProblem (aslam_backend / incremental_calibration).
        poseSplineDv: BSplinePoseDesignVariable (trajetoria do corpo/IMU de referencia).
        measurements: iteravel de dicts {timestamp, velocity(3), covariance(3x3),
                      velocity_valid, ...} (ver test/dvl/dvl_synthetic.py e o leitor de CSV).
        q_dvl_b_Dv: RotationQuaternionDv (C_dvl_b).
        r_dvl_b_Dv: EuclideanPointDv (r_dvl_b, lever arm).
        scale_dv: aopt.Scalar (escala s). Se estiver ativo, a escala e estimada (usa DvlVelocityError).
        time_offset: offset temporal DVL->spline [s] (somado ao timestamp).
        huber: largura do M-estimator de Huber (0 = sem robustez).

    Returns:
        (errors, n_added, n_skipped).
    """
    spline = poseSplineDv.spline()
    t_min, t_max = spline.t_min(), spline.t_max()
    estimate_scale = scale_dv.isActive()
    have_cpp_scale = hasattr(ket, "DvlVelocityError")

    mest = aopt.HuberMEstimator(huber) if huber > 0.0 else aopt.NoMEstimator()

    errors = []
    n_added = 0
    n_skipped = 0
    for m in measurements:
        if not m.get("velocity_valid", True):
            n_skipped += 1
            continue
        tk = float(m["timestamp"]) + time_offset
        if tk <= t_min or tk >= t_max:
            n_skipped += 1
            continue

        pred = predicted_velocity_expression(poseSplineDv, tk, q_dvl_b_Dv, r_dvl_b_Dv)
        invR = invR_from_covariance(m["covariance"], fallback_sigma=fallback_sigma)
        v_meas = np.asarray(m["velocity"], dtype=float).flatten()

        if estimate_scale:
            if not have_cpp_scale:
                raise NotImplementedError(
                    "Estimacao de escala requer ket.DvlVelocityError (tarefa 2.4 do plano).")
            err = ket.DvlVelocityError(v_meas, invR, pred, scale_dv.toExpression())
        else:
            s = scale_dv.toScalar()
            if abs(s - 1.0) > 1e-12:
                # escala fixa != 1: aplica via elementwiseMultiply com constante (spike 1.2)
                pred = pred.elementwiseMultiply(aopt.EuclideanExpression(np.array([s, s, s])))
            err = ket.EuclideanError(v_meas, invR, pred)

        err.setMEstimatorPolicy(mest)
        problem.addErrorTerm(err)
        errors.append(err)
        n_added += 1

    return errors, n_added, n_skipped
