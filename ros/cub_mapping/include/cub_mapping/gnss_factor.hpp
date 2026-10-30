#pragma once

#include <gtsam/navigation/GPSFactor.h>
#include <boost/serialization/export.hpp>

namespace cub_mapping {
// GPSFactorArm in the installed GTSAM has no serialization. Keep identical
// error/Jacobian semantics while explicitly persisting both antenna and fix.
class GnssArmFactor : public gtsam::NoiseModelFactorN<gtsam::Pose3> {
  using Base = gtsam::NoiseModelFactorN<gtsam::Pose3>;
 public:
  GnssArmFactor() = default;
  ~GnssArmFactor() override;
  GnssArmFactor(gtsam::Key key, const gtsam::Point3& fix,
                const gtsam::Point3& arm, const gtsam::SharedNoiseModel& noise)
      : Base(noise, key), fix_(fix), arm_(arm) {}
  using Base::evaluateError;
  gtsam::Vector evaluateError(const gtsam::Pose3& pose,
                             gtsam::OptionalMatrixType H) const override {
    return gtsam::GPSFactorArm(keys()[0], fix_, arm_, noiseModel()).evaluateError(pose, H);
  }
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::make_shared<GnssArmFactor>(*this);
  }
 private:
  gtsam::Point3 fix_ = gtsam::Point3::Zero(), arm_ = gtsam::Point3::Zero();
  friend class boost::serialization::access;
  template<class Archive> void serialize(Archive& ar, const unsigned int) {
    ar & boost::serialization::make_nvp("base", boost::serialization::base_object<Base>(*this));
    ar & BOOST_SERIALIZATION_NVP(fix_);
    ar & BOOST_SERIALIZATION_NVP(arm_);
  }
};
}
BOOST_CLASS_EXPORT_KEY(cub_mapping::GnssArmFactor)
