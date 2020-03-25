/**
 * Copyright (C)
 *
 * @file  Profiler.hpp
 *
 * @brief
 *
 *
 * @date  03.11.2016
 */
#ifndef UTILS_PROFILER_HPP
#define UTILS_PROFILER_HPP
#include <string>
#include <chrono>
#include <iostream>

namespace utilities
{
  struct Profiler
  {
    std::string                                    name;
    std::chrono::high_resolution_clock::time_point start;
    bool                                           mbVerbose = false;

    Profiler(std::string const& n, bool const verbose = false)
        : name(n), start(std::chrono::high_resolution_clock::now()), mbVerbose(verbose)
    {

    }

    ~Profiler()
    {
      if(mbVerbose)
      {
        using dura = std::chrono::duration<double>;
        auto d = std::chrono::high_resolution_clock::now() - start;
        std::cout << name << ": " << std::chrono::duration_cast<dura>(d).count() << " [sec]" << std::endl;
      }
    }

    double elapsedSec()
    {
      using dura = std::chrono::duration<double>;
      std::chrono::high_resolution_clock::time_point end       = std::chrono::high_resolution_clock::now();
      dura                                           time_span = std::chrono::duration_cast<dura>(end - start);
      return time_span.count();
    }

    double elapsedMS()
    {
      return elapsedSec() * 1000;
    }
  };
} // namespace utilities


#endif // UTILS_PROFILER_HPP
