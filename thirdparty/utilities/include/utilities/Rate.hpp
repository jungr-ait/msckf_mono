/******************************************************************************
* FILENAME:     Rate.hpp
* PURPOSE:      Rate
* AUTHOR:       jungr - Roland Jung
* VERSION:      v1.0.0
******************************************************************************//
#ifndef UTILS_RATE_HPP
#define UTILS_RATE_HPP
#include <thread>
#include <iostream>
#include <chrono>
#include <atomic>
#include <cstdint>

namespace utilities
{
  class Rate
  {
    public:
    /**
     * create object with desired frequency
     * @param hz desired frequency; 0 for maximum
     */
    Rate(double const& Hz);

    /**
     * sleep to achieve desired frequency
     */
    void sleep();

    std::int32_t getProcessTimeMS() const;

    void setRate(double const Hz);


    private:

    typedef std::chrono::high_resolution_clock Clock;

    Rate(const Rate&) = delete;

    Rate& operator=(const Rate&) = delete;

    std::atomic<std::uint32_t> mSleep_ms;
    Clock::time_point          mtStart;
    Clock::time_point          mtStop;
    std::chrono::milliseconds  mProcessTime;

  }; // class Rate

} // namespace utilities

#endif // UTILS_RATE_HPP
