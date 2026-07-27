#include <kalibr_errorterms/DvlVelocityError.hpp>

namespace kalibr_errorterms {

DvlVelocityError::DvlVelocityError(
    const Eigen::Vector3d & measurement,
    const Eigen::Matrix3d & invR,
    const aslam::backend::EuclideanExpression & predictedVelocity,
    const aslam::backend::ScalarExpression & scale)
    // EuclideanError computa (predicao - medicao); a predicao aqui e a velocidade escalada.
    : EuclideanError(measurement, invR, predictedVelocity * scale) {}

DvlVelocityError::~DvlVelocityError() {}

}  // namespace kalibr_errorterms
