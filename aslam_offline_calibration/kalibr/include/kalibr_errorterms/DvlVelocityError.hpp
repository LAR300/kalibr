#ifndef KALIBR_DVL_VELOCITY_ERROR_HPP
#define KALIBR_DVL_VELOCITY_ERROR_HPP

#include <aslam/backend/EuclideanExpression.hpp>
#include <aslam/backend/ScalarExpression.hpp>

#include <kalibr_errorterms/EuclideanError.hpp>

namespace kalibr_errorterms {

// Erro de velocidade do DVL: residuo = (scale * predictedVelocity) - measurement, ponderado por invR.
//
// A GEOMETRIA da velocidade predita (C_dvl_b * (C_b_w * v_w + w_b x r_b)) e montada em Python e
// entra ja como uma EuclideanExpression. Este error term apenas aplica a ESCALA escalar `scale`
// (design variable) a essa predicao, algo que a multiplicacao EuclideanExpression x ScalarExpression
// nao expoe em Python (ver .ai/specs/dvl-calibration/decisions.md D4/D7).
//
// Deriva de EuclideanError passando (predictedVelocity * scale) como a predicao; toda a maquinaria
// de residuo/jacobianas e reusada.
class DvlVelocityError : public EuclideanError {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  DvlVelocityError(const Eigen::Vector3d & measurement,
                   const Eigen::Matrix3d & invR,
                   const aslam::backend::EuclideanExpression & predictedVelocity,
                   const aslam::backend::ScalarExpression & scale);
  virtual ~DvlVelocityError();
};

} //namespace kalibr_errorterms

#endif /* KALIBR_DVL_VELOCITY_ERROR_HPP */
