#include <functional>

#include <CCC/Constants.h>

#include <BaselineWalkingController/BaselineWalkingController.h>
#include <BaselineWalkingController/FootManager.h>
#include <BaselineWalkingController/centroidal/CentroidalManagerPreviewControlZmp.h>

using namespace BWC;


void CentroidalManagerPreviewControlZmp::Configuration::load(const mc_rtc::Configuration & mcRtcConfig)
{
  CentroidalManager::Configuration::load(mcRtcConfig);

  mcRtcConfig("horizonDuration", horizonDuration);
  mcRtcConfig("horizonDt", horizonDt);
}

CentroidalManagerPreviewControlZmp::CentroidalManagerPreviewControlZmp(BaselineWalkingController * ctlPtr,
                                                                       const mc_rtc::Configuration & mcRtcConfig)
: CentroidalManager(ctlPtr, mcRtcConfig)
{
  config_.load(mcRtcConfig);
}

void CentroidalManagerPreviewControlZmp::reset()
{
  CentroidalManager::reset();

  pc_ = std::make_shared<CCC::PreviewControlZmp>(config_.refComZ, config_.horizonDuration, config_.horizonDt);

  firstIter_ = true;
}


//runMpc()函数方法实现
void CentroidalManagerPreviewControlZmp::runMpc()
{

  CCC::PreviewControlZmp::InitialParam initialParam;   //创建变量initialParam
  initialParam.pos = mpcCom_.head<2>();  //位置 pos为 mpcCom_ 的前两个元素。
  initialParam.vel = mpcComVel_.head<2>();//速度 vel为mpcComVel_ 的前两个元素

  if(firstIter_)//如果是第一次迭代
  {
    initialParam.acc.setZero();//加速度设为0
  }
  else//非第一次迭代
  {
    //使用线性倒力摆公式计算质心加速度
    initialParam.acc = CCC::constants::g / config_.refComZ * (mpcCom_ - plannedZmp_).head<2>();
  }

  //（1）调用planOnce函数生成质心轨迹，该函数接受了4个参数
  //                参数1：函数对象
  //                参数2：结构体initialParam
  //                参数3：ctl().t()，控制器的时间
  //                参数4： ctl().dt()，时间步长
  //（2）将计算的结果全部存储到plannedData
  Eigen::Vector2d plannedData =
      pc_->planOnce(std::bind(&CentroidalManagerPreviewControlZmp::calcRefData, this, std::placeholders::_1),
                    initialParam, ctl().t(), ctl().dt());

  //分别提取计算出的结果
  plannedZmp_ << plannedData, refZmp_.z();
  plannedForceZ_ = robotMass_ * CCC::constants::g;

  if(firstIter_)
  {
    firstIter_ = false;
  }
}



Eigen::Vector2d CentroidalManagerPreviewControlZmp::calcRefData(double t) const
{
  return ctl().footManager_->calcRefZmp(t).head<2>();
}
