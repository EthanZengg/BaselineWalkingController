#include <mc_tasks/MetaTaskLoader.h>

#include <BaselineWalkingController/tasks/FirstOrderImpedanceTask.h>

using namespace BWC;

FirstOrderImpedanceTask::FirstOrderImpedanceTask(const std::string & surfaceName,
                                                 const mc_rbdyn::Robots & robots,
                                                 unsigned int robotIndex,
                                                 double stiffness,
                                                 double weight)
: mc_tasks::force::ImpedanceTask(surfaceName, robots, robotIndex, stiffness, weight)
{
  const auto & robot = robots.robot(robotIndex);
  type_ = "firstOrderImpedance";
  name_ = "first_order_impedance_" + robots.robot(rIndex).name() + "_" + surfaceName;
}

void FirstOrderImpedanceTask::update(mc_solver::QPSolver & solver)
{
  double dt = solver.dt();

  // 1. Filter the measured wrench
  measuredWrench_ = robots.robot(rIndex).surfaceWrench(surface());
  lowPass_.update(measuredWrench_);
  filteredMeasuredWrench_ = lowPass_.eval();

  // 2. Compute the compliance acceleration
  sva::MotionVecd deltaCompVelWPrev = deltaCompVelW_;
  sva::PTransformd T_0_s(surfacePose().rotation());
  // deltaCompAccelW_ is represented in the world frame
  //   \Delta \ddot{p}_{cd} = - \frac{D}{M} \Delta \dot{p}_{cd} - \frac{K}{M} \Delta p_{cd})
  //   + \frac{K_f}{M} (f_m - f_d) where \Delta p_{cd} = p_c - p_d
  // See the Constructor description for the definition of symbols
  deltaCompVelW_ = T_0_s.invMul( // T_0_s.invMul transforms the MotionVecd value from surface to world frame
      sva::MotionVecd(
          // Compute in the surface frame because the impedance parameters and wrench gain are represented in the
          // surface frame
          gains().D().vector().cwiseInverse().cwiseProduct(
              // T_0_s transforms the MotionVecd value from world to surface frame
              -gains().K().vector().cwiseProduct((T_0_s * sva::transformVelocity(deltaCompPoseW_)).vector())
              + gains().wrench().vector().cwiseProduct((filteredMeasuredWrench_ - targetWrench_).vector()))));
  deltaCompAccelW_ = (deltaCompVelW_ - deltaCompVelWPrev) / dt;

  if(deltaCompAccelW_.linear().norm() > deltaCompAccelLinLimit_)
  {
    mc_rtc::log::warning("[FirstOrderImpedanceTask] Linear deltaCompAccel limited from {} to {}",
                         deltaCompAccelW_.linear().norm(), deltaCompAccelLinLimit_);
    deltaCompAccelW_.linear().normalize();
    deltaCompAccelW_.linear() *= deltaCompAccelLinLimit_;
  }
  if(deltaCompAccelW_.angular().norm() > deltaCompAccelAngLimit_)
  {
    mc_rtc::log::warning("[FirstOrderImpedanceTask] Angular deltaCompAccel limited from {} to {}",
                         deltaCompAccelW_.angular().norm(), deltaCompAccelAngLimit_);
    deltaCompAccelW_.angular().normalize();
    deltaCompAccelW_.angular() *= deltaCompAccelAngLimit_;
  }

  // 3. Compute the compliance pose and velocity by time integral
  // 3.1 Integrate velocity to pose
  sva::PTransformd T_0_deltaC(deltaCompPoseW_.rotation());
  // Represent the compliance velocity and acceleration in the deltaCompliance frame and scale by dt
  sva::MotionVecd mvDeltaCompVelIntegralC = T_0_deltaC * (dt * deltaCompVelW_);
  // sva::MotionVecd mvDeltaCompVelIntegralC = T_0_deltaC * (dt * (deltaCompVelW_ + 0.5 * dt * deltaCompAccelW_));
  // Convert the angular velocity to the rotation matrix through AngleAxis representation
  Eigen::AngleAxisd aaDeltaCompVelIntegralC(Eigen::Quaterniond::Identity());
  if(mvDeltaCompVelIntegralC.angular().norm() > 1e-6)
  {
    aaDeltaCompVelIntegralC =
        Eigen::AngleAxisd(mvDeltaCompVelIntegralC.angular().norm(), mvDeltaCompVelIntegralC.angular().normalized());
  }
  sva::PTransformd deltaCompVelIntegral(
      // Rotation matrix is transposed because sva::PTransformd uses the left-handed coordinates
      aaDeltaCompVelIntegralC.toRotationMatrix().transpose(), mvDeltaCompVelIntegralC.linear());
  // Since deltaCompVelIntegral is multiplied by deltaCompPoseW_, it must be represented in the deltaCompliance frame
  deltaCompPoseW_ = deltaCompVelIntegral * deltaCompPoseW_;
  // 3.2 Integrate acceleration to velocity
  // deltaCompVelW_ += dt * deltaCompAccelW_;

  if(deltaCompVelW_.linear().norm() > deltaCompVelLinLimit_)
  {
    mc_rtc::log::warning("[FirstOrderImpedanceTask] Linear deltaCompVel limited from {} to {}",
                         deltaCompVelW_.linear().norm(), deltaCompVelLinLimit_);
    deltaCompVelW_.linear().normalize();
    deltaCompVelW_.linear() *= deltaCompVelLinLimit_;
  }
  if(deltaCompVelW_.angular().norm() > deltaCompVelAngLimit_)
  {
    mc_rtc::log::warning("[FirstOrderImpedanceTask] Angular deltaCompVel limited from {} to {}",
                         deltaCompVelW_.angular().norm(), deltaCompVelLinLimit_);
    deltaCompVelW_.angular().normalize();
    deltaCompVelW_.angular() *= deltaCompVelAngLimit_;
  }

  if(deltaCompPoseW_.translation().norm() > deltaCompPoseLinLimit_)
  {
    mc_rtc::log::warning("[FirstOrderImpedanceTask] Linear deltaCompPose limited from {} to {}",
                         deltaCompPoseW_.translation().norm(), deltaCompPoseLinLimit_);
    deltaCompPoseW_.translation().normalize();
    deltaCompPoseW_.translation() *= deltaCompPoseLinLimit_;
  }
  Eigen::AngleAxisd aaDeltaCompRot(deltaCompPoseW_.rotation());
  if(aaDeltaCompRot.angle() > deltaCompPoseAngLimit_)
  {
    mc_rtc::log::warning("[FirstOrderImpedanceTask] Angular deltaCompPose limited from {} to {}",
                         aaDeltaCompRot.angle(), deltaCompPoseAngLimit_);
    aaDeltaCompRot.angle() = deltaCompPoseAngLimit_;
    deltaCompPoseW_.rotation() = aaDeltaCompRot.toRotationMatrix();
  }

  // 4. Update deltaCompPoseW_ in hold mode (See the hold method documentation for more information)
  if(hold_)
  {
    // Transform to target pose frame (see compliancePose implementation)
    sva::PTransformd T_0_d(targetPoseW_.rotation());
    // The previous compliancePose() is stored in mc_tasks::SurfaceTransformTask::target()
    deltaCompPoseW_ = T_0_d.inv() * mc_tasks::SurfaceTransformTask::target() * targetPoseW_.inv() * T_0_d;
  }

  // 5. Set compliance values to the targets of mc_tasks::SurfaceTransformTask
  mc_tasks::SurfaceTransformTask::refAccel(T_0_s
                                           * (targetAccelW_ + deltaCompAccelW_)); // represented in the surface frame
  mc_tasks::SurfaceTransformTask::refVelB(T_0_s * (targetVelW_ + deltaCompVelW_)); // represented in the surface frame
  mc_tasks::SurfaceTransformTask::target(compliancePose()); // represented in the world frame
}

namespace
{
static auto registered = mc_tasks::MetaTaskLoader::register_load_function(
    "firstOrderImpedance",
    [](mc_solver::QPSolver & solver, const mc_rtc::Configuration & config) {
      using Allocator = Eigen::aligned_allocator<BWC::FirstOrderImpedanceTask>;
      const auto robotIndex = robotIndexFromConfig(config, solver.robots(), "firstOrderImpedance");
      auto t = std::allocate_shared<BWC::FirstOrderImpedanceTask>(Allocator{}, config("surface"), solver.robots(),
                                                                  robotIndex);
      t->reset();
      t->load(solver, config);
      return t;
    });
}
