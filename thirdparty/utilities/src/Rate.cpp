#include <utilities/Rate.hpp>

namespace utilities
{

  Rate::Rate(const double& Hz)
  {
    setRate(Hz);
    mtStart = Clock::now();
  }

  void Rate::sleep()
  {
    mtStop       = Clock::now();
    mProcessTime = std::chrono::duration_cast<std::chrono::milliseconds>(mtStop - mtStart);
    std::int32_t sleep_ms = mSleep_ms - mProcessTime.count();

    if(sleep_ms > 0)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
    }

    mtStart = Clock::now();
  }

  int32_t Rate::getProcessTimeMS() const
  {
    return mProcessTime.count();
  }

  void Rate::setRate(const double Hz)
  {
    mSleep_ms = static_cast<std::uint32_t>((Hz == 0) ? 0 : 1000 / Hz);
  }

} // namespace utils
