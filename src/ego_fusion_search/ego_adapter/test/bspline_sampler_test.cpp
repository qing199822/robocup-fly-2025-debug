#include <stdexcept>

#include <gtest/gtest.h>

#include "ego_adapter/bspline_sampler.h"

namespace {

using ego_adapter::BsplineData;
using ego_adapter::BsplineSampler;
using ego_adapter::Vec3;

BsplineData straightQuadratic() {
  BsplineData spline;
  spline.order = 2;
  spline.traj_id = 9;
  spline.start_time = 10.0;
  spline.knots = {0.0, 0.0, 0.0, 1.0, 1.0, 1.0};
  spline.control_points = {
      Vec3{0.0, 0.0, 3.0},
      Vec3{1.0, 0.0, 3.0},
      Vec3{2.0, 0.0, 3.0},
  };
  return spline;
}

TEST(BsplineSampler, EvaluatesClampedQuadraticStartMiddleAndEnd) {
  const BsplineSampler sampler(straightQuadratic());

  const auto start = sampler.evaluate(10.0);
  const auto middle = sampler.evaluate(10.5);
  const auto end = sampler.evaluate(11.0);

  EXPECT_NEAR(0.0, start.position.x, 1e-12);
  EXPECT_NEAR(1.0, middle.position.x, 1e-12);
  EXPECT_NEAR(2.0, end.position.x, 1e-12);
  EXPECT_NEAR(3.0, start.position.z, 1e-12);
  EXPECT_NEAR(3.0, middle.position.z, 1e-12);
  EXPECT_NEAR(3.0, end.position.z, 1e-12);
  EXPECT_NEAR(2.0, start.velocity.x, 1e-12);
  EXPECT_NEAR(2.0, middle.velocity.x, 1e-12);
  EXPECT_NEAR(2.0, end.velocity.x, 1e-12);
}

TEST(BsplineSampler, RejectsTimesOutsideTrajectoryDomain) {
  const BsplineSampler sampler(straightQuadratic());
  EXPECT_THROW(sampler.evaluate(9.99), std::out_of_range);
  EXPECT_THROW(sampler.evaluate(11.01), std::out_of_range);
  EXPECT_DOUBLE_EQ(10.0, sampler.startTime());
  EXPECT_DOUBLE_EQ(11.0, sampler.endTime());
}

TEST(BsplineSampler, RejectsMalformedSplineData) {
  BsplineData malformed = straightQuadratic();
  malformed.knots.pop_back();
  EXPECT_THROW(BsplineSampler{malformed}, std::invalid_argument);

  malformed = straightQuadratic();
  malformed.traj_id = -1;
  const BsplineSampler signed_id(malformed);
  EXPECT_EQ(-1, signed_id.trajectoryId());
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
