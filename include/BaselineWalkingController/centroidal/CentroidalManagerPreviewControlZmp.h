#pragma once

#include <CCC/PreviewControlZmp.h>

#include <BaselineWalkingController/CentroidalManager.h>

namespace BWC
{

//利用预冠控制生成质心轨迹

//定义子类，公有继承CentroidalManager
class CentroidalManagerPreviewControlZmp : public CentroidalManager
{
public:
  
  //定义配置结构体。
  struct Configuration : public CentroidalManager::Configuration
  {
    //成员变量
    double horizonDuration = 2.0;//时长
    double horizonDt = 0.005;//时间间隔
    //结构体成员函数load声明，用于加载 mc_rtc 配置，重写了基类的同名函数
    virtual void load(const mc_rtc::Configuration & mcRtcConfig) override;
  };

  //该类的构造函数，有两个参数列表
  CentroidalManagerPreviewControlZmp(BaselineWalkingController * ctlPtr,
                                     const mc_rtc::Configuration & mcRtcConfig = {});


  //声明 reset函数，控制器reset时调用
  virtual void reset() override;

  //返回当前对象的配置对象的常量引用
  inline virtual const Configuration & config() const override
  {
    return config_;
  }

protected:
  //返回当前对象的配置对象的非常量引用
  inline virtual Configuration & config() override
  {
    return config_;
  }

  //声明runMpc函数，根据mpcCom_ 和mpcComVel_.计算质心轨迹、plannedZmp_、plannedForceZ_ 
  virtual void runMpc() override;


//用于判断是否假设 CoM的 Z 坐标是恒定的
  inline virtual bool isConstantComZ() const override
  {
    return true;
  }

  //声明函数calcRefData，用于MPC的参考数据
  Eigen::Vector2d calcRefData(double t) const;

protected:
  //! Configuration
  Configuration config_;

  //! Preview control
  std::shared_ptr<CCC::PreviewControlZmp> pc_;

  //! Whether it is the first iteration
  bool firstIter_ = true;
};
} // namespace BWC
