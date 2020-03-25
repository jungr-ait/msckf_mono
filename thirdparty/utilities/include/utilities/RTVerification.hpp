/******************************************************************************
* FILENAME:     RTVerification.hpp
* PURPOSE:
* AUTHOR:       Roland Jung
*
******************************************************************************/
#ifndef RTVERIFICATION_HPP
#define RTVERIFICATION_HPP
#include <stdexcept>
#include <sstream>
#include <iostream>
#include <iomanip>


namespace utilities
{
  class RTV
  {
    public:

    static bool Verify(bool condition,
                       const char *expression,
                       const char *msg,
                       bool expected,
                       const char *file,
                       int line_num,
                       bool bFail = false)
    {
      if(condition != expected)
      {
        std::stringstream ss;
        ss << "Error: " << expression << " is not " << std::boolalpha << expected << "! " << file << ": " << line_num
            << "\n";

        if(msg)
        {
          ss << "--> msg:'" << msg << "'\n";
        }

        if(bFail)
        {
          throw std::runtime_error(ss.str());
        }
        else
        {
          std::cout << ss.str();
          return false;
        }
      }

      return true;
    }

  }; // class RTV

} // namespace utilities


// only verification of condition
#define RTV_EXPECT_TRUE_(condition) \
  utilities::RTV::Verify(condition, #condition, 0, true, __FILE__, __LINE__)

#define RTV_EXPECT_TRUE(condition, msg) \
  utilities::RTV::Verify(condition, #condition, msg, true, __FILE__, __LINE__)

// throws exception if invalid
#define RTV_EXPECT_TRUE_THROW(condition, msg) \
  utilities::RTV::Verify(condition, #condition, msg, true, __FILE__, __LINE__, true)

// returns the result of the verification
#define RTV_EXPECT_TRUE_RET(condition, msg) \
  return utilities::RTV::Verify(condition, #condition, msg, true, __FILE__, __LINE__);

#define RTV_EXPECT_TRUE_RET_(condition) \
  return utilities::RTV::Verify(condition, #condition, 0, true, __FILE__, __LINE__);

// returns false if invalid
#define RTV_EXPECT_TRUE_CONDRET(condition, msg) \
  if(!utilities::RTV::Verify(condition, #condition, msg, true, __FILE__, __LINE__)) return false;


#define RTV_EXPECT_TRUE_CONDRET_(condition) \
  if(!utilities::RTV::Verify(condition, #condition, 0, true, __FILE__, __LINE__)) return false;

#define RTV_EXPECT_TRUE_CONDRET2_(condition) \
  if(!utilities::RTV::Verify(condition, #condition, 0, true, __FILE__, __LINE__)) return;

#endif // RTVERIFICATION_H
