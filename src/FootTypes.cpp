#include <mc_rtc/logging.h>

#include <BaselineWalkingController/FootTypes.h>

using namespace BWC;


//字符串转枚举
Foot BWC::strToFoot(const std::string & footStr)
{
  if(footStr == "Left")
  {
    return Foot::Left;
  }
  else if(footStr == "Right")
  {
    return Foot::Right;
  }
  else
  {
    mc_rtc::log::error_and_throw("[strToFoot] Unsupported Foot name: {}", footStr);
  }
}

//返回对脚
Foot BWC::opposite(const Foot & foot)
{
  if(foot == Foot::Left)
  {
    return Foot::Right;
  }
  else // if(footStr == "Right")
  {
    return Foot::Left;
  }
}

//左脚返回正，右脚返回负
int BWC::sign(const Foot & foot)
{
  if(foot == Foot::Left)
  {
    return 1;
  }
  else // if(footStr == "Right")
  {
    return -1;
  }
}

//函数重载，用于Foot枚举值转化为字符串
std::string std::to_string(const Foot & foot)
{
  if(foot == Foot::Left)
  {
    return std::string("Left");
  }
  else if(foot == Foot::Right)
  {
    return std::string("Right");
  }
  else
  {
    mc_rtc::log::error_and_throw("[to_string] Unsupported foot: {}", std::to_string(static_cast<int>(foot)));
  }
}


//函数重载，用于枚举值转化为字符串
std::string std::to_string(const SupportPhase & phase)
{
  if(phase == SupportPhase::DoubleSupport)
  {
    return std::string("DoubleSupport");
  }
  else if(phase == SupportPhase::LeftSupport)
  {
    return std::string("LeftSupport");
  }
  else if(phase == SupportPhase::RightSupport)
  {
    return std::string("RightSupport");
  }
  else
  {
    mc_rtc::log::error_and_throw("[to_string] Unsupported support phase: {}", std::to_string(static_cast<int>(phase)));
  }
}
