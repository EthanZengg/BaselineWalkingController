#include <sys/syscall.h>

#include <mc_tasks/CoMTask.h>
#include <mc_tasks/FirstOrderImpedanceTask.h>
#include <mc_tasks/MetaTaskLoader.h>
#include <mc_tasks/OrientationTask.h>

#include <BaselineWalkingController/BaselineWalkingController.h>
#include <BaselineWalkingController/CentroidalManager.h>
#include <BaselineWalkingController/FootManager.h>
#include <BaselineWalkingController/centroidal/CentroidalManagerDdpZmp.h>
#include <BaselineWalkingController/centroidal/CentroidalManagerFootGuidedControl.h>
#include <BaselineWalkingController/centroidal/CentroidalManagerIntrinsicallyStableMpc.h>
#include <BaselineWalkingController/centroidal/CentroidalManagerPreviewControlZmp.h>

using namespace BWC;


//BaselineWalkingController类的构造函数(继承自mc_control::fsm::Controller类)，并接受rm、dt和 _config参数
BaselineWalkingController::BaselineWalkingController(mc_rbdyn::RobotModulePtr rm,  //智能指针指向机器人的模型模块
                                                     double dt,                                                                                                           //控制器的时间步长
                                                     const mc_rtc::Configuration & _config)                                                 //对象的常引用，返回控制器的配置信息
: mc_control::fsm::Controller(rm, dt, _config)             //调用基类构造函数，子类可以在构造过程中初始化从基类继承的成员变量和功能
{


//config() 是 BaselineWalkingController 类的成员函数，config() 返回一个 mc_rtc::Configuration 对象，可以通过括号运算符进行访问
//获取机器人的配置项，这里首先访问yaml文件中robot部分，然后再访问robot().module().name，即JVRC或者walker，并将配置文件存储未rconfig
  auto rconfig = config()("robots")(robot().module().name);  


//如果空的话提示输出
  if(rconfig.empty())
  {
    mc_rtc::log::error_and_throw("[BaselineWalkingController] {} section is empty, please provide a configuration",
                                 robot().module().name);
  }


  // 将上述读取的配置文件载入到控制器中
  config().load(rconfig);


  // 获取而外的配置项
  auto overwriteConfigList = config()("OverwriteConfigList", mc_rtc::Configuration()); //获取 OverwriteConfigList 的配置对象，如果该配置项不存在，则使用默认值 mc_rtc::Configuration()。
  auto overwriteConfigKeys = config()("OverwriteConfigKeys", std::vector<std::string>{}); //获取 OverwriteConfigKeys 的字符串数组，如果该配置项不存在，则使用空的默认值 std::vector<std::string>{}。

//遍历 overwriteConfigKeys 中的每个键名，并检查是否在 overwriteConfigList 中存在相应的配置项
  for(const auto & overwriteConfigKey : overwriteConfigKeys)
  {
    if(!overwriteConfigList.has(overwriteConfigKey))  //如果某个键名在 overwriteConfigKeys 中存在，但在 overwriteConfigList 中不存在对应的配置项
    {
      mc_rtc::log::error_and_throw(
          "[BaselineWalkingController] {} in OverwriteConfigKeys but not in OverwriteConfigList", overwriteConfigKey);//抛出错误
    }
    config().load(overwriteConfigList(overwriteConfigKey));//如果存在，则将对应的配置项加载到控制器的配置中
  }

//控制器的名称从配置文件中加载到控制器的成员变量 name_ 中
  config()("controllerName", name_);


  // 设置质心任务
  if(config().has("CoMTask")) //配置对象中是如果存在名为 "CoMTask" 的配置项
  {
    comTask_ = mc_tasks::MetaTaskLoader::load<mc_tasks::CoMTask>(solver(), config()("CoMTask"));//添加mc_rtc任务
    comTask_->name("CoMTask");//名称设置
  }
  else
  {
    mc_rtc::log::warning("[BaselineWalkingController] CoMTask configuration is missing.");
  }

  // 设置OrientationTask 任务
  if(config().has("BaseOrientationTask"))
  {
    baseOriTask_ = mc_tasks::MetaTaskLoader::load<mc_tasks::OrientationTask>(solver(), config()("BaseOrientationTask"));
    baseOriTask_->name("BaseOriTask");
  }
  else
  {
    mc_rtc::log::warning("[BaselineWalkingController] BaseOrientationTask configuration is missing.");
  }

//设置脚部任务
  if(config().has("FootTaskList"))
  {
    for(const auto & footTaskConfig : config()("FootTaskList"))   // 遍历 "FootTaskList" 配置项的每个元素
    {
      Foot foot = strToFoot(footTaskConfig("foot"));  // 将 "foot" 字段的值转换为 Foot 枚举类型，并赋值给 foot
    
      // 从 footTaskConfig 加载相应的配置值，并使用 MetaTaskLoader 创建一个 FirstOrderImpedanceTask 脚部任务对象
      // 将创建的脚部任务对象插入到 footTasks_ 容器中，键为 foot
      footTasks_.emplace(
          foot, mc_tasks::MetaTaskLoader::load<mc_tasks::force::FirstOrderImpedanceTask>(solver(), footTaskConfig));

      //将该脚部任务对象的名称设置为 "FootTask_" + foot 的字符串
      footTasks_.at(foot)->name("FootTask_" + std::to_string(foot));
    }
  }
  else
  {
    mc_rtc::log::warning("[BaselineWalkingController] FootTaskList configuration is missing.");
  }


  //设置脚部管理器（FootManager）
  if(config().has("FootManager"))
  {
    footManager_ = std::make_shared<FootManager>(this, config()("FootManager")); // 创建智能指针指向配置文件FootManager
  }
  else
  {
    mc_rtc::log::warning("[BaselineWalkingController] FootManager configuration is missing.");
  }

  // 设置质心管理器（CentroidalManager）
  if(config().has("CentroidalManager"))
  {
    std::string centroidalManagerMethod = config()("CentroidalManager")("method", std::string("")); // 获取 "CentroidalManager" 配置项中的 "method" 字段值，默认为空字符串
    if(centroidalManagerMethod == "PreviewControlZmp") //若method为PreviewControlZmp
    {
      centroidalManager_ = std::make_shared<CentroidalManagerPreviewControlZmp>(this, config()("CentroidalManager"));
    }
    else if(centroidalManagerMethod == "DdpZmp")
    {
      centroidalManager_ = std::make_shared<CentroidalManagerDdpZmp>(this, config()("CentroidalManager"));
    }
    else if(centroidalManagerMethod == "FootGuidedControl")
    {
      centroidalManager_ = std::make_shared<CentroidalManagerFootGuidedControl>(this, config()("CentroidalManager"));
    }
    else if(centroidalManagerMethod == "IntrinsicallyStableMpc")
    {
      centroidalManager_ =
          std::make_shared<CentroidalManagerIntrinsicallyStableMpc>(this, config()("CentroidalManager"));
    }
    else
    {
      mc_rtc::log::error_and_throw("[BaselineWalkingController] Invalid centroidalManagerMethod: {}.",
                                   centroidalManagerMethod);
    }
  }
  else
  {
    mc_rtc::log::warning("[BaselineWalkingController] CentroidalManager configuration is missing.");
  }



  // Setup anchor
  setDefaultAnchor();

  mc_rtc::log::success("[BaselineWalkingController] Constructed.");
}



void BaselineWalkingController::reset(const mc_control::ControllerResetData & resetData)
{
 
  mc_control::fsm::Controller::reset(resetData); //调用基类的reset函数

  enableManagerUpdate_ = false;  // 禁用管理器更新

  // Print message to set priority
  long tid = static_cast<long>(syscall(SYS_gettid));
  mc_rtc::log::info("[BaselineWalkingController] TID is {}. Run the following command to set high priority:\n  sudo "
                    "renice -n -20 -p {}",
                    tid, tid);
  mc_rtc::log::info("[BaselineWalkingController] You can check the current priority by the following command:\n  ps -p "
                    "`pgrep choreonoid` -o pid,tid,args,ni,pri,wchan m");

  mc_rtc::log::success("[BaselineWalkingController] Reset.");
}


bool BaselineWalkingController::run()
{
  t_ += dt();// 更新时间

  if(enableManagerUpdate_)
  {
    // 更新管理器
    footManager_->update();
    centroidalManager_->update();
  }

  return mc_control::fsm::Controller::run();// 调用基类的 run 函数
}

void BaselineWalkingController::stop()
{
  // 清理任务
  solver().removeTask(comTask_);
  solver().removeTask(baseOriTask_);
  for(const auto & foot : Feet::Both)
  {
    solver().removeTask(footTasks_.at(foot));
  }

  // 清理管理器
  footManager_->stop();
  footManager_.reset();
  centroidalManager_->stop();
  centroidalManager_.reset();

   // 清理锚点
  setDefaultAnchor();

  // 调用基类的 stop 函数
  mc_control::fsm::Controller::stop();
}

void BaselineWalkingController::setDefaultAnchor()
{
  // 构建锚点名称
  std::string anchorName = "KinematicAnchorFrame::" + robot().name();
  if(datastore().has(anchorName))
  {
    // 如果锚点已存在，则移除
    datastore().remove(anchorName);
  }
  // 创建锚点回调函数
  datastore().make_call(anchorName, [this](const mc_rbdyn::Robot & robot) {
    return sva::interpolate(robot.surfacePose(footManager_->surfaceName(Foot::Left)),
                            robot.surfacePose(footManager_->surfaceName(Foot::Right)), 0.5);
  });
}
