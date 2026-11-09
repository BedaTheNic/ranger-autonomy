// Controller lifecycle adapted from Navigation2 Humble MPPI controller.cpp.
// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
// Modifications: feasible sampled Ackermann controls and post-filter constraints.
// SPDX-License-Identifier: Apache-2.0
#include <algorithm>
#include <cmath>
#include <atomic>
#include <exception>
#include <optional>
#include "navigation_geometry.hpp"
#include "reversal_alignment.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nav2_mppi_controller/controller.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace ranger_nav_plugins
{
// Fresh wheel observations are valid in every steering mode. A pivot has no
// finite Ackermann curvature; use a neutral initial estimate while the base
// aligns the wheels, instead of incorrectly declaring the feedback stale.
inline std::optional<double> steeringCurvature(const sensor_msgs::msg::JointState & msg,
  double wheelbase,double track)
{
  double numerator=0,denominator=0;
  for(const std::string joint:{"fl_steering_wheel","fr_steering_wheel","rl_steering_wheel","rr_steering_wheel"}) {
    auto found=std::find(msg.name.begin(),msg.name.end(),joint);
    if(found==msg.name.end()) {return std::nullopt;}
    const auto i=std::distance(msg.name.begin(),found);
    if(size_t(i)>=msg.position.size() || !std::isfinite(msg.position[i])) {return std::nullopt;}
    const double x=joint[0]=='f'?wheelbase/2:-wheelbase/2;
    const double y=joint[1]=='l'?track/2:-track/2;
    const double s=std::sin(msg.position[i]),c=std::cos(msg.position[i]);
    const double coefficient=x*c+y*s;
    numerator+=coefficient*s;denominator+=coefficient*coefficient;
  }
  return denominator>1e-6?numerator/denominator:0.0;
}
// Humble constrains the optimized mean, but normally simulates independent
// unconstrained vx/wz noise. Project the sampled controls BEFORE prediction so
// collision, path and goal critics all evaluate motions this car can execute.
class FeasibleAckermannModel : public mppi::MotionModel
{
public:
  FeasibleAckermannModel(std::shared_ptr<mppi::AckermannMotionModel> original,
    const mppi::models::ControlConstraints & limits)
  : original_(std::move(original)), limits_(limits) {}
  bool isHolonomic() override {return false;}
  void setDirection(int direction) {direction_ = direction;}
  void setSteering(float curvature) {initial_curvature_ = curvature;}
  float minimumRadius() const {return original_->getMinTurningRadius();}
  void setDynamics(float dt, float acceleration, float deceleration,
    float steering_rate, float wheelbase)
  {
    dt_ = dt; acceleration_ = acceleration; deceleration_ = deceleration;
    steering_rate_ = steering_rate; half_wheelbase_ = wheelbase / 2.0f;
  }
  void predict(mppi::models::State & state) override
  {
    const float radius = original_->getMinTurningRadius();
    if (!std::isfinite(radius) || radius <= 0 || limits_.vx_min > 0.0f) {
      throw std::runtime_error("Feasible Ackermann model requires radius > 0 and vx_min <= 0");
    }
    for (size_t i = 0; i < state.cvx.shape(0); ++i) {
      float previous_v = state.vx(i, 0);
      float previous_steering = std::atan(
        std::clamp(initial_curvature_, -1.0f / radius, 1.0f / radius) * half_wheelbase_);
      float actual_steering = previous_steering;
      for (size_t j = 0; j < state.cvx.shape(1); ++j) {
        auto & vx = state.cvx(i, j);
        auto & wz = state.cwz(i, j);
        vx = std::clamp(vx, direction_ > 0 ? 0.0f : limits_.vx_min,
          direction_ < 0 ? 0.0f : limits_.vx_max);
        const float bound = std::min(limits_.wz, std::abs(vx) / radius);
        wz = std::clamp(wz, -bound, bound);
        if (dt_ > 0.0f) {
          // Predict actuator-feasible controls, so critics score the same
          // gradual steering/speed changes that the platform can execute.
          const float lower = previous_v - (previous_v >= 0 ? deceleration_ : acceleration_) * dt_;
          const float upper = previous_v + (previous_v >= 0 ? acceleration_ : deceleration_) * dt_;
          vx = std::clamp(vx, lower, upper);
          const float target_curvature = std::abs(vx) > 1e-5f ?
            std::clamp(wz / vx, -1.0f / radius, 1.0f / radius) : initial_curvature_;
          float steering = std::atan(target_curvature * half_wheelbase_);
          // At rest the base can first align its wheels. Do not trap the
          // optimizer at zero speed by demanding that it drive with the OLD
          // steering angle to request the NEW one. Subsequent horizon commands
          // change smoothly; rollouts below explicitly include alignment time.
          if (j > 0 || std::abs(state.vx(i, 0)) > 0.02f) {
            steering = std::clamp(steering, previous_steering - steering_rate_ * dt_,
              previous_steering + steering_rate_ * dt_);
          }
          wz = vx * std::tan(steering) / half_wheelbase_;
          wz = std::clamp(wz, -limits_.wz, limits_.wz);
          previous_v = vx;
          previous_steering = steering;
          actual_steering += std::clamp(steering - actual_steering, -dt_, dt_);
          if (j + 1 < state.cvx.shape(1)) {
            const bool aligned = std::abs(steering - actual_steering) <= 0.02f;
            state.vx(i, j + 1) = aligned ? vx : 0.0f;
            state.wz(i, j + 1) = aligned ? wz : 0.0f;
          }
        }
      }
    }
    if (dt_ <= 0.0f) {
      // The controls above are already projected. Jazzy's base predictor also
      // acceleration-clamps them and would overwrite them a second time.
      for (size_t i = 0; i < state.cvx.shape(0); ++i) {
        for (size_t j = 1; j < state.cvx.shape(1); ++j) {
          state.vx(i, j) = state.cvx(i, j - 1);
          state.wz(i, j) = state.cwz(i, j - 1);
        }
      }
    }
  }
  void applyConstraints(mppi::models::ControlSequence & sequence) override
  {
    original_->applyConstraints(sequence);
  }
private:
  // Keep the original model alive: its radius is bound to the upstream dynamic
  // parameter handler. Do not leave callbacks referring to a destroyed object.
  std::shared_ptr<mppi::AckermannMotionModel> original_;
  const mppi::models::ControlConstraints & limits_;
  int direction_{0};
  float initial_curvature_{0}, dt_{0}, acceleration_{0.3}, deceleration_{0.4};
  float steering_rate_{0.15}, half_wheelbase_{0.23};
};

class FeasibleOptimizer : public mppi::Optimizer
{
public:
  void initialize(rclcpp_lifecycle::LifecycleNode::WeakPtr parent, const std::string & name,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap, mppi::ParametersHandler * parameters)
  {
    mppi::Optimizer::initialize(parent, name, costmap, parameters);
    auto original = std::dynamic_pointer_cast<mppi::AckermannMotionModel>(motion_model_);
    if (!original) {
      throw std::runtime_error("This controller requires Ackermann MPPI");
    }
    feasible_model_ = std::make_shared<FeasibleAckermannModel>(original, settings_.constraints);
    auto get = parameters->getParamGetter(name);
    get(acceleration_, "tracking_acceleration", 0.3f);
    get(deceleration_, "tracking_deceleration", 0.4f);
    get(steering_rate_, "tracking_steering_rate", 0.15f);
    get(wheelbase_, "tracking_wheelbase", 0.46f);
    if (!std::isfinite(acceleration_) || !std::isfinite(deceleration_) ||
      !std::isfinite(steering_rate_) || !std::isfinite(wheelbase_) ||
      std::min({acceleration_, deceleration_, steering_rate_, wheelbase_}) <= 0.0f)
    {
      throw std::runtime_error("Tracking dynamics must be positive");
    }
    feasible_model_->setDynamics(settings_.model_dt, acceleration_, deceleration_, steering_rate_, wheelbase_);
    motion_model_ = feasible_model_;
#ifdef RANGER_NAV2_JAZZY
    motion_model_->initialize(settings_.constraints, settings_.model_dt);
#endif
  }
  int direction() const {return previous_direction_;}
  mppi::models::ControlConstraints limits() const {return settings_.constraints;}
  void setSteering(float curvature)
  {
    feasible_model_->setDynamics(settings_.model_dt, acceleration_, deceleration_, steering_rate_, wheelbase_);
    feasible_model_->setSteering(curvature);
  }
  geometry_msgs::msg::TwistStamped evalControl(const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & speed, const nav_msgs::msg::Path & path,
#ifdef RANGER_NAV2_JAZZY
    const geometry_msgs::msg::Pose & goal,
#endif
    nav2_core::GoalChecker * checker)
  {
    // PathHandler truncates at the next Reeds-Shepp inversion. Sample only
    // that segment's direction, avoiding a near-zero mean of forward/reverse
    // candidates near a goal. The global planner still chooses either mode.
    if (path.poses.empty()) {throw std::runtime_error("Empty tracking path");}
    int direction = previous_direction_;
    if (!path.poses.empty()) {
      const auto & first = path.poses.front().pose;
      const auto & q = first.orientation;
      const double yaw = std::atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z));
      for (size_t i = 1; i < path.poses.size(); ++i) {
        const auto & next = path.poses[i].pose.position;
        const double projection = (next.x-first.position.x)*std::cos(yaw) +
          (next.y-first.position.y)*std::sin(yaw);
        if (std::abs(projection) > 1e-4) {direction = projection > 0 ? 1 : -1; break;}
      }
    }
    if (direction != previous_direction_) {reset(); previous_direction_ = direction;}
    feasible_model_->setDirection(direction);
    seedFromReference(path, planar(pose.pose), speed, direction);
#ifdef RANGER_NAV2_JAZZY
    return mppi::Optimizer::evalControl(pose, speed, path, goal, checker);
#else
    return mppi::Optimizer::evalControl(pose, speed, path, checker);
#endif
  }
private:
  // A slow previous solution centres the next batch of noisy controls around
  // another crawl. Give MPPI a rolling path-following reference on clear arcs.
  // This changes where it samples, never the command returned after scoring.
  void seedFromReference(const nav_msgs::msg::Path & plan, Pose2 robot,
    const geometry_msgs::msg::Twist & measured, int direction)
  {
    if (plan.poses.size() < 2 || direction * measured.linear.x < -0.02) {
      return;
    }
    const auto reference = points(plan);
    const auto footprint = costmap_ros_->getRobotFootprint();
    const auto & limits = settings_.constraints;
    const double speed_limit = direction > 0 ? limits.vx_max : -limits.vx_min;
    const double dt = settings_.model_dt;
    const double radius = feasible_model_->minimumRadius();
    double magnitude = std::min(std::abs(measured.linear.x), speed_limit);
    size_t cursor = 0;
    auto seed = control_sequence_;
    for (size_t i = 0; i < seed.vx.size(); ++i) {
      const auto projection = project(reference, robot, cursor);
      cursor = projection.index;
      const auto target = lookahead(reference, projection, 0.45);
      auto guide = pursuit(robot, target, true, direction, radius, speed_limit);
      const double curvature = guide.linear.x == 0.0 ? 0.0 : guide.angular.z / guide.linear.x;
      // The transformed subpath ends at the next cusp or goal. Taper as the
      // lookahead shortens and never accelerate toward a point behind that end.
      double requested = std::abs(guide.linear.x);
      if (std::abs(curvature) > 1e-6) {
        requested = std::min(requested, limits.wz / std::abs(curvature));
      }
      magnitude = std::clamp(requested, magnitude - deceleration_ * dt,
        magnitude + acceleration_ * dt);
      guide.linear.x = direction * magnitude;
      guide.angular.z = guide.linear.x * curvature;
      const auto next = integrate(robot, guide, dt);
      if (!pathFree({robot, next}, *costmap_, footprint)) {
        return;  // Keep the previous optimum, including its braking solution.
      }
      // Retain some of the optimizer's previous avoidance and steering history.
      seed.vx(i) = 0.75 * guide.linear.x + 0.25 * seed.vx(i);
      seed.wz(i) = 0.75 * guide.angular.z + 0.25 * seed.wz(i);
      robot = next;
    }
    control_sequence_ = std::move(seed);
  }
  std::shared_ptr<FeasibleAckermannModel> feasible_model_;
  int previous_direction_{1};
  float acceleration_, deceleration_, steering_rate_, wheelbase_;
};

class ReversalPathHandler : public mppi::PathHandler
{
public:
  bool atEndOfIncomingLeg() const
  {
    return enforce_path_inversion_ && inversion_locale_ != 0 &&
           global_plan_up_to_inversion_.poses.size() == 1;
  }

  bool needsCuspApproach(const Pose2 & robot, const nav_msgs::msg::Path & local_plan,
    int direction, double radius) const
  {
    if (!atEndOfIncomingLeg() || local_plan.poses.size() != 1) {
      return false;
    }
    const auto cusp = planar(local_plan.poses.back().pose);
    const double remaining = distance(robot, cusp);
    const double along = (cusp.x - robot.x) * std::cos(robot.a) +
      (cusp.y - robot.y) * std::sin(robot.a);
    // No more incoming geometry. The car cannot gain the missing heading
    // over the remaining approach distance, or has already passed the cusp.
    const double error = std::abs(wrap(cusp.a - robot.a));
    return (remaining <= inversion_xy_tolerance_ && error > inversion_yaw_tolerance + remaining / radius) ||
           (remaining > inversion_xy_tolerance_ && direction * along <= 0.0);
  }
};

class AckermannController : public nav2_core::Controller
{
public:
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, const std::shared_ptr<tf2_ros::Buffer> tf,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap) override
  {
    parent_ = parent; name_ = name; costmap_ = costmap;
    parameters_ = std::make_unique<mppi::ParametersHandler>(parent);
    auto node = parent.lock(); clock_ = node->get_clock(); last_call_ = clock_->now();
    auto get = parameters_->getParamGetter(name);
    get(visualize_, "visualize", false);
    get(reset_period_, "reset_period", 1.0);
    optimizer_.initialize(parent, name, costmap, parameters_.get());
    get(track_, "tracking_track", 0.412);
    get(wheelbase_, "tracking_wheelbase", 0.46);
    if (!std::isfinite(track_) || !std::isfinite(wheelbase_) || track_ <= 0 || wheelbase_ <= 0) {
      throw std::runtime_error("Tracking geometry must be positive and finite");
    }
    steering_sub_ = node->create_subscription<sensor_msgs::msg::JointState>("joint_states", 10,
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        auto curvature=steeringCurvature(*msg,wheelbase_,track_);
        if (curvature) {
          curvature_.store(*curvature);
          steering_stamp_.store(clock_->now().nanoseconds());
        }
      });
    path_.initialize(parent, name, costmap, tf, parameters_.get());
    visualizer_.on_configure(parent, name, costmap->getGlobalFrameID(), parameters_.get());
    RCLCPP_INFO(node->get_logger(), "MPPI with feasible Ackermann samples and path direction configured");
  }
  void cleanup() override
  {
    optimizer_.shutdown(); visualizer_.on_cleanup(); parameters_.reset();
  }
  void activate() override {visualizer_.on_activate(); parameters_->start();}
  void deactivate() override {reference_.poses.clear();visualizer_.on_deactivate();}
  void setPlan(const nav_msgs::msg::Path & plan) override
  {
    if(sameStampedPath(reference_,plan)) {return;}
    path_.setPath(plan);reference_=plan;assist_.reset();
  }
  void setSpeedLimit(const double & limit, const bool & percentage) override
  {
    optimizer_.setSpeedLimit(limit, percentage);
  }
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & speed, nav2_core::GoalChecker * checker) override
  {
    if (clock_->now() - last_call_ > rclcpp::Duration::from_seconds(reset_period_)) {
      optimizer_.reset();
    }
    last_call_ = clock_->now();
    if (steering_stamp_.load() == 0 ||
      clock_->now().nanoseconds() - steering_stamp_.load() > 500000000)
    {
      throw std::runtime_error("Missing or stale steering joint feedback");
    }
    std::lock_guard<std::mutex> parameter_lock(*parameters_->getLock());
    auto plan = path_.transformPath(pose);
    if (path_.needsCuspApproach(planar(pose.pose), plan, optimizer_.direction(), nodeRadius())) {
      throw ReversalAlignment(plan.poses.back());
    }
    optimizer_.setSteering(curvature_.load());
    auto grid = costmap_->getCostmap();
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> map_lock(*grid->getMutex());
    geometry_msgs::msg::TwistStamped command;command.header=pose.header;
    std::exception_ptr optimizer_error;
    try {command = optimizer_.evalControl(pose, speed, plan,
#ifdef RANGER_NAV2_JAZZY
      path_.getTransformedGoal(pose.header.stamp).pose,
#endif
      checker);}
    catch(const std::runtime_error & error) {
      if(std::string(error.what())!="Optimizer fail to compute path") {throw;}
      optimizer_error=std::current_exception();
    }
    const auto robot=planar(pose.pose);
    if (optimizer_error || assist_.needed(robot,clock_->now().seconds(),std::abs(command.twist.linear.x))) {
      const auto reference=points(plan);
      auto projection=project(reference,robot);
      auto target=lookahead(reference,projection,.30);
      const double radius=nodeRadius();
      const auto limits=optimizer_.limits();
      const double speed_limit=optimizer_.direction()>0?limits.vx_max:-limits.vx_min;
      const double acceleration=parent_.lock()->get_parameter(name_ + ".tracking_acceleration").as_double();
      auto candidate=pursuit(robot,target,true,optimizer_.direction(),radius,
        std::min({.08,speed_limit,std::abs(speed.linear.x)+acceleration*.1}));
      if (plan.poses.size() == 1 && candidate.linear.x == 0.0 && speed_limit > 0.0) {
        // Progress assistance already observed a stall (or MPPI exhausted its
        // candidates). Repeating zero until the progress timeout cannot put
        // the endpoint back in front of the car in its current gear.
        if (path_.atEndOfIncomingLeg()) {throw ReversalAlignment(plan.poses.back());}
        throw AckermannTrackingError("Final Ackermann approach exhausted before reaching the goal pose");
      }
      if(collisionFree(robot,candidate,*grid,costmap_->getRobotFootprint())) {
        command.twist=candidate;
        optimizer_error=nullptr;
        RCLCPP_INFO_THROTTLE(parent_.lock()->get_logger(),*clock_,3000,
          "Progress assist: collision-checked Ackermann tracking on retained path");
      }
    }
    if(optimizer_error) {std::rethrow_exception(optimizer_error);}
    if (visualize_) {
      visualizer_.add(optimizer_.getGeneratedTrajectories(), "Feasible candidates");
      visualizer_.add(optimizer_.getOptimizedTrajectory(), "Optimal trajectory"
#ifdef RANGER_NAV2_JAZZY
        , command.header.stamp
#endif
        );
      visualizer_.visualize(std::move(plan));
    }
    auto node = parent_.lock();
    const double radius = node->get_parameter(name_ + ".AckermannConstraints.min_turning_r").as_double();
    const double vmin = node->get_parameter(name_ + ".vx_min").as_double();
    const double vmax = node->get_parameter(name_ + ".vx_max").as_double();
    const double wmax = node->get_parameter(name_ + ".wz_max").as_double();
    auto & v = command.twist;
    if (!std::isfinite(radius) || radius <= 0 || !std::isfinite(vmin) || vmin > 0 || !std::isfinite(vmax) || vmax <= 0 ||
      !std::isfinite(wmax) || wmax <= 0 || !std::isfinite(v.linear.x) || !std::isfinite(v.angular.z))
    {throw std::runtime_error("Invalid Ackermann command or limits");}
    // Humble applies Savitzky-Golay after optimizer constraints. Restore the
    // same feasibility bounds before velocity interpolation; the final output
    // guard also checks the interpolation across direction changes.
    const auto limits=optimizer_.limits();
    v.linear.x = std::clamp(v.linear.x, std::max(vmin,double(limits.vx_min)),
      std::min(vmax,double(limits.vx_max))); v.linear.y = 0.0;
    const double bound = std::min({wmax,double(limits.wz),std::abs(v.linear.x) / radius});
    v.angular.z = std::clamp(v.angular.z, -bound, bound);
    return command;
  }
private:
  double nodeRadius() const
  {return parent_.lock()->get_parameter(name_ + ".AckermannConstraints.min_turning_r").as_double();}
  ProgressAssist assist_;
  nav_msgs::msg::Path reference_;
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_;
  std::string name_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Time last_call_;
  bool visualize_{false}; double reset_period_{1.0};
  std::unique_ptr<mppi::ParametersHandler> parameters_;
  FeasibleOptimizer optimizer_;
  ReversalPathHandler path_;
  mppi::TrajectoryVisualizer visualizer_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr steering_sub_;
  std::atomic<float> curvature_{0.0f};
  std::atomic<int64_t> steering_stamp_{0};
  double wheelbase_{0.46}, track_{0.412};
};
}
PLUGINLIB_EXPORT_CLASS(ranger_nav_plugins::AckermannController, nav2_core::Controller)
