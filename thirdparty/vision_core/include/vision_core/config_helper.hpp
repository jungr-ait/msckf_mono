/******************************************************************************
* FILENAME:     config_helper
* PURPOSE:      %{Cpp:License:ClassName}
* AUTHOR:       jungr - Roland Jung
* MAIL:         Roland.Jung@ait.ac.at
* VERSION:      v1.0.0
* CREATION:     9.10.2017
*
*  Copyright (C) 2017 Austrian Institute of Technologies GmbH - AIT
*  All rights reserved. See the LICENSE file for details.
******************************************************************************/
#ifndef CONFIG_HELPER_HPP
#define CONFIG_HELPER_HPP
#include <opencv2/core/core.hpp>

namespace vision_core {

  /**
  * @brief helper functions to handle yaml files
  */
  namespace config_helper {

    static inline bool loadNode(cv::FileNode &fn, const cv::FileStorage &fs, const std::string &name, bool const required = false)
    {
      fn = fs[name];

      if(fn.empty())
      {
        if(required)
        {
          std::cerr << "node " << name << " does not exist in config file";
        }

        return false;
      }

      return true;
    }

    template<typename T>
    static inline void get_if(cv::FileNode const &fn, std::string const &name, T &dst)
    {
      if(fn[name].isNamed())
      {
        dst = static_cast<T>(fn[name]);
      }
    }

    static inline void get_if_boolean(const cv::FileNode &fn, const std::string &name, bool &dst)
    {
      if(!fn[name].isNamed())
      {
        return;
      }
      int val = -1;
      get_if(fn, name, val);
      if(val != 1 && val != 0)
      {
        std::cerr << "Node " << name << " is boolean (0/1), given " << val << std::endl;
        return;
      }
      dst = static_cast<bool>(val);
    }

    static inline void get_if_uint32(const cv::FileNode &fn, const std::string &name, std::uint32_t &dst)
    {
      int val = static_cast<int>(dst);
      get_if(fn, name, val);
      if(val < 0)
      {
        std::cerr << "Node " << name << " is defined to be not less then zero, given " << val << std::endl;
        return;
      }
      dst = static_cast<uint32_t>(val);
    }
  }
}

#endif // CONFIG_HELPER_HPP
