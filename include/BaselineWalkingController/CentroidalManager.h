#pragma once

#include <mc_rtc/gui/StateBuilder.h>
#include <mc_rtc/log/Logger.h>

#include <BaselineWalkingController/FootTypes.h>

namespace mc_rbdyn
{
class Robot;
}

namespace ForceColl
{
class Contact;
class WrenchDistribution;
} // namespace ForceColl

namespace BWC
{
class BaselineWalkingController;

//定义类：Centroidal manager。其作用是根据指定的ZMP曲线和传感器测量值计算质心目标

class CentroidalManager
{
public:

  //配置结构体，用于配置参数
  struct Configuration
  {
    
    std::string name = "CentroidalManager"; //名称，表示质心管理器的名称
    std::string method = "";//方法，表示质心管理器使用的方法。

  
    bool useActualStateForMpc = false; //是否使用实际状态进行模型预测控制（MPC）。

    bool enableZmpFeedback = true; //是否启动DCM反馈

    bool enableComZFeedback = true; //是否启用质心Z轴位置反馈控制

    /** \brief  DCM反馈的增益值
        It must be greater than 1 to be stable.
    */
    double dcmGainP = 2.0;

    double zmpVelGain = 0.1; //ZMP前向速度增益

    double comZGainP = 100.0;   //质心Z轴位置的比例反馈增益

    double comZGainD = 10.0;    //质心Z轴位置的导数反馈增益

    double refComZ = 0.5;    //参考质心高度 [m]

    bool useTargetPoseForControlRobotAnchorFrame = true;     //否使用目标表面位姿作为控制机器人锚点框架的参考

    bool useActualComForWrenchDist = true;    //是否使用实际质心位置进行力矩分配

    mc_rtc::Configuration wrenchDistConfig;    //! Configuration for wrench distribution

    virtual void load(const mc_rtc::Configuration & mcRtcConfig);     // 加载mc_rtc配置
  };



public:
  /** \brief 构造函数
      \param ctlPtr pointer to controller 控制器指针
      \param mcRtcConfig mc_rtc configuration mc_rtc配置文件
   */
  CentroidalManager(BaselineWalkingController * ctlPtr, const mc_rtc::Configuration & mcRtcConfig = {});


  // 声明质心管理管理器reset函数，在控制器重置时被调用
  virtual void reset();


   // 声明质心管理管理器update函数，在每个控制周期被调用，用于执行质心管理器的更新逻辑。
  virtual void update();

 
   // 声明质心管理管理器update函数，在停止控制器时被调用
  virtual void stop();

  //定义虚函数config()，返回类型为const Configuration &，在派生类中使用config()来对具体的Configuration配置信息的访问
  virtual const Configuration & config() const = 0;

  /** \brief Add entries to the GUI. */
  virtual void addToGUI(mc_rtc::gui::StateBuilder & gui);

  /** \brief Remove entries from the GUI. */
  virtual void removeFromGUI(mc_rtc::gui::StateBuilder & gui);

  /** \brief Add entries to the logger. */
  virtual void addToLogger(mc_rtc::Logger & logger);

  /** \brief Remove entries from the logger. */
  virtual void removeFromLogger(mc_rtc::Logger & logger);

  void setAnchorFrame(); //声明函数，用于设定锚点框架

protected:
  
  inline const BaselineWalkingController & ctl() const  //返回控制器常量引用
  {
    return *ctlPtr_;
  }

  inline BaselineWalkingController & ctl() //返回控制器的引用
  {
    return *ctlPtr_;
  }

  virtual Configuration & config() = 0;//返回控制器引用

  virtual void runMpc() = 0;//运行MPC来规划质心轨迹。它从mpcCom_和mpcComVel_计算plannedZmp_和plannedForceZ_。

  virtual bool isConstantComZ() const = 0; //是否假设质心Z坐标固定不变

  // 声明函数，用来计算锚点框架
  // 传入参数：robot 机器人
  sva::PTransformd calcAnchorFrame(const mc_rbdyn::Robot & robot) const;


  // 声明函数，作用：根据提供的力矩列表、ZMP平面高度和ZMP平面法线计算ZMP，
  //  传入参数：力距列表、ZMP平面高度、ZMP平面法线
  Eigen::Vector3d calcZmp(const std::unordered_map<Foot, sva::ForceVecd> & wrenchList,
                          double zmpPlaneHeight = 0,
                          const Eigen::Vector3d & zmpPlaneNormal = Eigen::Vector3d::UnitZ()) const;



 //声明函数，用于计算质心的加速度。将其在基类中声明为虚函数的目的是为了在派生类中进行重写，以支持扩展的质心-零力矩点（CoM-ZMP）模型，例如考虑操纵力等其他因素
  virtual Eigen::Vector3d calcPlannedComAccel() const;

protected:
  
  BaselineWalkingController * ctlPtr_ = nullptr; //指向BaselineWalkingController的指针

  double robotMass_ = 0;//机器人的质量，以千克（kg）为单位

  Eigen::Vector3d mpcCom_ = Eigen::Vector3d::Zero(); //MPC初始状态的质心位置

  Eigen::Vector3d mpcComVel_ = Eigen::Vector3d::Zero();//MPC初始状态的质心速度

  Eigen::Vector3d refZmp_ = Eigen::Vector3d::Zero();//参考ZMP

  Eigen::Vector3d plannedZmp_ = Eigen::Vector3d::Zero(); //有MPC计算出的ZMP曲线

  double plannedForceZ_ = 0; //由MPC计算得出的力在Z轴方向上的分量

  Eigen::Vector3d controlZmp_ = Eigen::Vector3d::Zero();//带有反馈控制的ZMP

  double controlForceZ_ = 0;  //是带有反馈控制的力在Z轴方向上的分量。

  //! Wrench distribution
  std::shared_ptr<ForceColl::WrenchDistribution> wrenchDist_;//创建共享指针，指向WrenchDistribution，用于进行力矩分配

  std::unordered_map<Foot, std::shared_ptr<ForceColl::Contact>> contactList_;//个无序映射容器，用于存储与脚步相关的接触对象
};
} // namespace BWC
