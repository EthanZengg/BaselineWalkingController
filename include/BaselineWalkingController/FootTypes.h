#pragma once

#include <set>

#include <mc_rtc/Configuration.h>

namespace BWC
{

//定义枚举类型Foot
enum class Foot
{
  //左脚枚举常量为0。右脚枚举常量为1
  Left = 0,
  Right
};

namespace Feet
{
//定义了存储类型为Foot的常量合集Both，该合集存储了两个枚举值
const std::set<Foot> Both = {Foot::Left, Foot::Right};
} 

//声明函数：strToFoot。该函数用于将字符串转化为脚部的枚举值
Foot strToFoot(const std::string & footStr);

//声明函数：opposite。该函数用于获取给定脚相对的脚
Foot opposite(const Foot & foot);


// 获取脚部的符号。对于左脚返回正数，对于右脚返回负数。
int sign(const Foot & foot);


//定义了一个枚举类型 ：支撑阶段，其包含三个枚举值，分别是双脚支撑，左脚支撑，右脚支撑
enum class SupportPhase
{
  //双脚支撑的枚举值为0
  DoubleSupport = 0,

  //左脚支撑的枚举值为1
  LeftSupport,

  //右脚支撑的枚举值为2
  RightSupport
};

//定义结构体Footstep，表示脚步信息的结构体
struct Footstep
{
  /** \brief 构造函数
      \param _foot    脚部
      \param _pose   脚步姿态
      \param _transitStartTime  ZMP切换开始时间
      \param _swingStartTime  脚步摆动开始时间
      \param _swingEndTime   脚步摆动结束时
      \param _transitEndTime   ZMP切换结束时间
      \param _swingTrajConfig   脚步摆动轨迹的配置信息

      \note The following relation must hold: _transitStartTime < _swingStartTime < _swingEndTime < _transitEndTime.
  */
  Footstep(Foot _foot,
           const sva::PTransformd & _pose,
           double _transitStartTime = 0,
           double _swingStartTime = 0,
           double _swingEndTime = 0,
           double _transitEndTime = 0,
           const mc_rtc::Configuration & _swingTrajConfig = {})
  : foot(_foot), pose(_pose), transitStartTime(_transitStartTime), swingStartTime(_swingStartTime),
    swingEndTime(_swingEndTime), transitEndTime(_transitEndTime), swingTrajConfig(_swingTrajConfig)
  {
  }

  //! Foot
  Foot foot;

  //! Foot pose
  sva::PTransformd pose;

  //! Time to start ZMP transition
  double transitStartTime;

  //! Time to start swinging the foot
  double swingStartTime;

  //! Time to end swinging the foot
  double swingEndTime;

  //! Time to end ZMP transition
  double transitEndTime;

  //! Configuration for swing trajectory
  mc_rtc::Configuration swingTrajConfig = {};
};
} // namespace BWC



//在 std 命名空间下重载了两个函数，分别用于将 Foot 枚举和 SupportPhase 枚举转换为字符串
namespace std
{

std::string to_string(const BWC::Foot & foot);
std::string to_string(const BWC::SupportPhase & phase);

} // namespace std
