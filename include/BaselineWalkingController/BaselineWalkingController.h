#pragma once

#include <mc_control/fsm/Controller.h>

#include <BaselineWalkingController/FootTypes.h>

namespace mc_tasks
{
struct CoMTask;
struct OrientationTask;

namespace force
{
struct FirstOrderImpedanceTask;
}
} // namespace mc_tasks

namespace BWC
{
class FootManager;
class CentroidalManager;

/** \brief Humanoid walking controller with various baseline methods. */
struct BaselineWalkingController : public mc_control::fsm::Controller
{
public:
  /** \brief Constructor. */
  BaselineWalkingController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & _config);

  /** \brief Reset a controller.

      This method is called when starting the controller.
   */
  void reset(const mc_control::ControllerResetData & resetData) override;

  /** \brief Run a controller.

      This method is called every control period.
   */
  bool run() override;

  /** \brief Stop a controller.

      This method is called when stopping the controller.
   */
  void stop() override;

  /** \brief Get controller name. */
  inline const std::string & name() const
  {
    return name_;
  }

  /** \brief Get current time. */
  inline double t() const noexcept
  {
    return t_;
  }

  /** \brief Get timestep. */
  inline double dt() const
  {
    return solver().dt();
  }

  /** \brief Set default anchor. */
  void setDefaultAnchor();

public:
  //! CoM task
  std::shared_ptr<mc_tasks::CoMTask> comTask_;

  //! Base link orientation task
  std::shared_ptr<mc_tasks::OrientationTask> baseOriTask_;
// 1.创建了名为baseOriTask_的智能指针变量
// 2.指针所指向对象的类型为mc_tasks 命名空间中的 OrientationTask ，是用于控制base link方向的任务
// 3.OrientationTask用于控制坐标系的方向


  //! Foot tasks
  std::unordered_map<Foot, std::shared_ptr<mc_tasks::force::FirstOrderImpedanceTask>> footTasks_;
// 1.无序映射容器，通过键可以快速寻找对应的值
// 2.对应关系： Foot ->std::shared_ptr<mc_tasks::force::FirstOrderImpedanceTask>
// 3.键表示机器人的脚步，值是一个共享指针，表示对应脚的力控任务
// 4.footTasks_是映射，使用方法：ctl().footTasks_.at(foot)，输出foot相应的力控任务


  //! Foot manager
  std::shared_ptr<FootManager> footManager_;

  //! Centroidal manager
  std::shared_ptr<CentroidalManager> centroidalManager_;

  //! Whether to enable manager update
  bool enableManagerUpdate_ = false;

protected:
  //! Controller name
  std::string name_ = "BWC";

  //! Current time [sec]
  double t_ = 0;
};
} // namespace BWC
