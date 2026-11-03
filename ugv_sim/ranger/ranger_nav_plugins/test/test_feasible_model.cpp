// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include "../src/ackermann_controller.cpp"

TEST(ReversalHandoff, OnlyRequestsAlignmentAtAnUnreleasedCusp)
{
  struct Handler : ranger_nav_plugins::ReversalPathHandler
  {
    void prepare(const nav_msgs::msg::Path & leg, unsigned int inversion) {
      enforce_path_inversion_ = true;
      inversion_locale_ = inversion;
      inversion_xy_tolerance_ = .04;
      inversion_yaw_tolerance = .08;
      global_plan_up_to_inversion_ = leg;
    }
  } handler;
  nav_msgs::msg::Path leg;
  geometry_msgs::msg::PoseStamped cusp;
  cusp.pose.orientation.w = 1.0;
  leg.poses.push_back(cusp);
  handler.prepare(leg, 9);
  EXPECT_TRUE(handler.needsCuspApproach({.01,0,.19}, leg, 1, .8));
  EXPECT_FALSE(handler.needsCuspApproach({.01,0,.04}, leg, 1, .8));
  EXPECT_FALSE(handler.needsCuspApproach({-.12,0,.19}, leg, 1, .8));
  EXPECT_TRUE(handler.needsCuspApproach({.06,0,.19}, leg, 1, .8));
  handler.prepare(leg, 0);
  EXPECT_FALSE(handler.needsCuspApproach({.01,0,.19}, leg, 1, .8));
  leg.poses.push_back(cusp);
  handler.prepare(leg, 9);
  EXPECT_FALSE(handler.needsCuspApproach({.01,0,.19}, leg, 1, .8));
}

TEST(SteeringFeedback, PivotAndLateralSteeringRemainFresh)
{
  sensor_msgs::msg::JointState m;
  m.name={"fl_steering_wheel","fr_steering_wheel","rl_steering_wheel","rr_steering_wheel"};
  for(auto xy:std::vector<std::pair<double,double>>{{.23,.206},{.23,-.206},{-.23,.206},{-.23,-.206}}) {
    m.position.push_back(std::atan2(xy.first,-xy.second));
  }
  auto pivot=ranger_nav_plugins::steeringCurvature(m,.46,.412);
  ASSERT_TRUE(pivot);EXPECT_DOUBLE_EQ(*pivot,0);
  std::fill(m.position.begin(),m.position.end(),M_PI_2);
  auto lateral=ranger_nav_plugins::steeringCurvature(m,.46,.412);
  ASSERT_TRUE(lateral);EXPECT_NEAR(*lateral,0,1e-9);
  m.position.clear();
  for(auto xy:std::vector<std::pair<double,double>>{{.23,.206},{.23,-.206},{-.23,.206},{-.23,-.206}}) {
    m.position.push_back(std::atan2(.8*xy.first,1-.8*xy.second));
  }
  EXPECT_NEAR(*ranger_nav_plugins::steeringCurvature(m,.46,.412),.8,1e-9);
}
TEST(SteeringFeedback, MissingOrNonfiniteWheelFeedbackIsRejected)
{
  sensor_msgs::msg::JointState m;
  m.name={"fl_steering_wheel","fr_steering_wheel","rl_steering_wheel","rr_steering_wheel"};
  m.position={0,0,0};EXPECT_FALSE(ranger_nav_plugins::steeringCurvature(m,.46,.412));
  m.position.push_back(NAN);EXPECT_FALSE(ranger_nav_plugins::steeringCurvature(m,.46,.412));
}

TEST(FeasibleAckermann, SamplesAndPredictionShareLimits)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("feasible_model_test");
  node->declare_parameter("FollowPath.AckermannConstraints.min_turning_r", 0.8);
  mppi::ParametersHandler parameters(node);
  auto original = std::make_shared<mppi::AckermannMotionModel>(&parameters, "FollowPath");
  mppi::models::ControlConstraints limits;
  limits.vx_max = 0.2f; limits.vx_min = 0.0f; limits.vy = 0.0f; limits.wz = 0.25f;
  ranger_nav_plugins::FeasibleAckermannModel model(original, limits);
  mppi::models::State state; state.reset(100, 10);
  for (size_t i = 0; i < 100; ++i) {
    state.vx(i, 0) = 0.03f; state.wz(i, 0) = -0.01f;
    for (size_t j = 0; j < 10; ++j) {
      state.cvx(i, j) = (static_cast<float>(i) - 30.0f) / 100.0f;
      state.cwz(i, j) = (static_cast<float>(j) - 5.0f) / 4.0f;
    }
  }
  model.predict(state);
  for (size_t i = 0; i < 100; ++i) {
    EXPECT_FLOAT_EQ(state.vx(i, 0), 0.03f);
    EXPECT_FLOAT_EQ(state.wz(i, 0), -0.01f);
    for (size_t j = 0; j < 10; ++j) {
      EXPECT_GE(state.cvx(i, j), 0.0f);
      EXPECT_LE(state.cvx(i, j), 0.2f);
      EXPECT_LE(std::abs(state.cwz(i, j)) * 0.8f, state.cvx(i, j) + 1e-6f);
      if (j > 0) {
        EXPECT_FLOAT_EQ(state.vx(i, j), state.cvx(i, j - 1));
        EXPECT_FLOAT_EQ(state.wz(i, j), state.cwz(i, j - 1));
      }
    }
  }
  limits.vx_max = 0.1f;
  model.predict(state);
  EXPECT_LE(xt::amax(state.cvx)(), 0.1f);
  limits.vx_min = -0.15f;
  state.cvx.fill(-0.4f); state.cwz.fill(0.5f);
  model.predict(state);
  EXPECT_FLOAT_EQ(state.cvx(0, 0), -0.15f);
  EXPECT_NEAR(state.cwz(0, 0), 0.15f / 0.8f, 1e-6f);
  EXPECT_FLOAT_EQ(state.vx(0, 1), state.cvx(0, 0));
  EXPECT_FLOAT_EQ(state.wz(0, 1), state.cwz(0, 0));
  model.setDirection(1);
  model.predict(state);
  EXPECT_FLOAT_EQ(state.cvx(0, 0), 0.0f);
  EXPECT_FLOAT_EQ(state.cwz(0, 0), 0.0f);
  model.setDirection(-1);
  state.cvx.fill(0.2f); state.cwz.fill(0.2f);
  model.predict(state);
  EXPECT_FLOAT_EQ(state.cvx(0, 0), 0.0f);
  EXPECT_FLOAT_EQ(state.cwz(0, 0), 0.0f);
  // Tracking rollouts limit speed and virtual steering changes before scoring.
  limits.vx_max = 0.2f;
  model.setDirection(1);
  model.setDynamics(0.1f, 0.3f, 0.4f, 0.15f, 0.46f);
  model.setSteering(0.5f);
  for (size_t i = 0; i < 100; ++i) {state.vx(i, 0) = 0.1f;}
  state.cvx.fill(0.2f); state.cwz.fill(-0.25f);
  model.predict(state);
  for (size_t i = 0; i < 100; ++i) {
    float previous_v = 0.1f, previous_steering = std::atan(0.5f * 0.23f);
    for (size_t j = 0; j < 10; ++j) {
      const float v = state.cvx(i, j), w = state.cwz(i, j);
      const float steering = std::atan(w / v * 0.23f);
      EXPECT_LE(v - previous_v, 0.030001f);
      EXPECT_LE(previous_v - v, 0.040001f);
      EXPECT_LE(std::abs(steering - previous_steering), 0.015001f);
      EXPECT_LE(std::abs(w) * 0.8f, v + 1e-6f);
      previous_v = v; previous_steering = steering;
    }
  }
  // A stationary robot may request a different steering angle, but prediction
  // must wait for the steering actuators before crediting wheel motion.
  model.setSteering(1.0f);
  for (size_t i = 0; i < 100; ++i) {state.vx(i, 0) = 0.0f;}
  state.cvx.fill(0.2f); state.cwz.fill(-0.25f);
  model.predict(state);
  EXPECT_GT(state.cvx(0, 0), 0.0f);
  EXPECT_LT(state.cwz(0, 0), 0.0f);
  EXPECT_FLOAT_EQ(state.vx(0, 1), 0.0f);
  EXPECT_FLOAT_EQ(state.wz(0, 1), 0.0f);
  EXPECT_GT(state.vx(0, 9), 0.0f);
  node.reset(); rclcpp::shutdown();
}
