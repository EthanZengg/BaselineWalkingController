#include <mc_rtc/gui/Button.h>
#include <mc_tasks/CoMTask.h>
#include <mc_tasks/FirstOrderImpedanceTask.h>
#include <mc_tasks/OrientationTask.h>

#include <BaselineWalkingController/BaselineWalkingController.h>
#include <BaselineWalkingController/CentroidalManager.h>
#include <BaselineWalkingController/FootManager.h>
#include <BaselineWalkingController/states/InitialState.h>


using namespace BWC;



//start函数
void InitialState::start(mc_control::fsm::Controller & _ctl)
{
  State::start(_ctl);

  phase_ = 0;

  // 设置一个“Start”按钮，按下后，phase_变为1，并进入run函数
  ctl().gui()->addElement({ctl().name()}, mc_rtc::gui::Button("Start", [this]() { phase_ = 1; }));

  output("OK");
}



//run函数
bool InitialState::run(mc_control::fsm::Controller &)
{
  //若phase_为0，即按下Start按钮
  if(phase_ == 0)
  {
    // Auto start
    if(config_.has("configs") && config_("configs").has("autoStartTime")
       && ctl().t() > static_cast<double>(config_("configs")("autoStartTime")))
    {
      phase_ = 1;
    }
  }

  //若phase_为1
  if(phase_ == 1)
  {
    phase_ = 2;
    // 清除"Start"按钮
    ctl().gui()->removeElement({ctl().name()}, "Start");

    // 重置和添加任务（Reset and add tasks）
    ctl().comTask_->reset();  
    std::cout<<" 质心任务重置完成"<<std::endl;
    ctl().solver().addTask(ctl().comTask_);  
    std::cout<<" 完成质心任务的添加"<<std::endl;

    ctl().baseOriTask_->reset();
     std::cout<<" 基座方向任务重置完成"<<std::endl;
    ctl().solver().addTask(ctl().baseOriTask_);  
    std::cout<<" 完成基座方向任务的添加"<<std::endl;

    //3.添加左右脚任务
    for(const auto & foot : Feet::Both)    //定义循环变量foot，遍历枚举类型Feet::Both中的定义值
    {
      ctl().footTasks_.at(foot)->reset();
      ctl().solver().addTask(ctl().footTasks_.at(foot));    
    }
    std::cout<<" 完成左右脚任务的添加"<<std::endl;

    // 设置任务刚度（Setup task stiffness interpolation）
    comTaskStiffness_ = ctl().comTask_->dimStiffness();  
    baseOriTaskStiffness_ = ctl().baseOriTask_->dimStiffness();
    footTasksStiffness_ = ctl().footManager_->config().footTaskGain.stiffness;
    constexpr double stiffnessInterpDuration = 1.0; // [sec]
    stiffnessRatioFunc_ = std::make_shared<TrajColl::CubicInterpolator<double>>(
        std::map<double, double>{{ctl().t(), 0.0}, {ctl().t() + stiffnessInterpDuration, 1.0}});
        


    // Reset managers
    ctl().footManager_->reset();

    ctl().centroidalManager_->reset();

    ctl().enableManagerUpdate_ = true;


    // Setup anchor frame
    ctl().centroidalManager_->setAnchorFrame();

    // Add GUI of managers
    ctl().footManager_->addToGUI(*ctl().gui());
    ctl().centroidalManager_->addToGUI(*ctl().gui());
  }

  //如果phase为2
  else if(phase_ == 2)
  {
    phase_ = 3;

    // Add logger of managers
    // Considering the possibility that logger entries assume that variables are set in the manager's update method,
    // it is safe to call the update method once and then add the logger
    ctl().footManager_->addToLogger(ctl().logger());
    ctl().centroidalManager_->addToLogger(ctl().logger());
  }

  // Interpolate task stiffness
  if(stiffnessRatioFunc_)
  {
    if(ctl().t() <= stiffnessRatioFunc_->endTime())
    {
      double stiffnessRatio = (*stiffnessRatioFunc_)(ctl().t());
      ctl().comTask_->stiffness(stiffnessRatio * comTaskStiffness_);
      ctl().baseOriTask_->stiffness(stiffnessRatio * baseOriTaskStiffness_);
      for(const auto & foot : Feet::Both)
      {
        ctl().footTasks_.at(foot)->stiffness(stiffnessRatio * footTasksStiffness_);
      }
    }
    else
    {
      stiffnessRatioFunc_.reset();
    }
  }
  return (phase_ == 3 && !stiffnessRatioFunc_);
}


//teardown函数
void InitialState::teardown(mc_control::fsm::Controller &) {}

EXPORT_SINGLE_STATE("BWC::Initial", InitialState)
