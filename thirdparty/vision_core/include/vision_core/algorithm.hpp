/******************************************************************************
* FILENAME:     algoritms
* PURPOSE:      %{Cpp:License:ClassName}
* AUTHOR:       jungr - Roland Jung
* MAIL:         Roland.Jung@ait.ac.at
* VERSION:      v1.0.0
* CREATION:     4.10.2017
*
*  Copyright (C) 2017 AIT Austrian Institute of Technology GmbH
*  All rights reserved. See the LICENSE file for details.
******************************************************************************/
#ifndef VISION_CORE_ALGORIhTM_HPP
#define VISION_CORE_ALGORIhTM_HPP
#include <set>
#include <vector>
#include <algorithm>
#include <iterator>

/**
* @brief collection of computer vision and geometry core functions
*/
namespace vision_core
{
  /**
  * @brief collection of vector algebra functions
  */
  namespace algorithm
  {

    // std::vector<int> v1, v2, v3;
    // for(int i=0; i < 10; i++)
    // {
    //   v1.push_back(i);
    //   v2.push_back(i*2);
    // }
    // v3 = std::move(vectorDiff(v2,v1));
    // print(v1);    //  0 1 2 3 4 5 6 7 8 9
    // print(v2);    //  0 2 4 6 8 10 12 14 16 18
    // print(v3);    //  10 12 14 16 18
    // http://stackoverflow.com/questions/5225820/compare-two-vectors-c
    /**
     * @brief removes the elements of vector v2 from vector v1 if present
     *
     * @param v1 original vector
     * @param v2 candidates to be removed if present in v1
     * @return resulting vector v1 without elements given in v2
     */
    template<typename T>
    std::vector<T> vectorDiff(std::vector<T> const& v1, std::vector<T> const& v2)
    {
      std::set<T>    s1(v1.begin(), v1.end());
      std::set<T>    s2(v2.begin(), v2.end());
      std::vector<T> v3;
      auto           it = std::set_difference(s1.begin(), s1.end(), s2.begin(), s2.end(), std::back_inserter(v3));
      return v3;
    }

    /**
     * @brief intersection set of two vectors in the sense of set theory
     *
     * @param v1 first pool of elements
     * @param v2 second pool of elements
     * @return elements contained in both pools
     */
    template<typename T>
    std::vector<T> vectorIntersection(std::vector<T> const& v1, std::vector<T> const& v2)
    {
      std::set<T>    s1(v1.begin(), v1.end());
      std::set<T>    s2(v2.begin(), v2.end());
      std::vector<T> v3;
      auto           it = std::set_intersection(s1.begin(), s1.end(), s2.begin(), s2.end(), std::back_inserter(v3));
      return v3;
    }

    /**
     * @brief the inverse set of the intersection, i.e. the elements exclusively be present once are returned
     *
     * @param v1 first pool of elements
     * @param v2 second pool of elements
     * @return elements exclusively occouring once (but possibly multiple times per pool, containted only once)
     */
    template<typename T>
    std::vector<T> vectorUniques(std::vector<T> const& v1, std::vector<T> const& v2)
    {
      std::set<T>    s1(v1.begin(), v1.end());
      std::set<T>    s2(v2.begin(), v2.end());
      std::vector<T> v3;
      auto           it = std::set_symmetric_difference(s1.begin(),
          s1.end(),
          s2.begin(),
          s2.end(),
          std::back_inserter(v3));
      return v3;
    }

    /**
     * @brief the union of the two sets in the sense of set theory
     *
     * @param v1 first pool of elements
     * @param v2 second pool of elements
     * @return the set of all elements in v1 and v2
     */
    template<typename T>
    std::vector<T> vectorUnion(std::vector<T> const& v1, std::vector<T> const& v2)
    {
      std::set<T>    s1(v1.begin(), v1.end());
      std::set<T>    s2(v2.begin(), v2.end());
      std::vector<T> v3;
      auto           it = std::set_union(s1.begin(), s1.end(), s2.begin(), s2.end(), std::back_inserter(v3));
      return v3;
    }

    /**
     * @brief get median of sorted vector
     *
     * @param v vector with sorted elements
     * @return median element
     */
    template<typename T>
    T inline getMedianPreSorted(std::vector<T>& v)
    {
      if(v.size() > 0)
      {
        return v[(v.size() - 1) / 2];
      }
      else
      {
        return 0;
      }
    }

    /**
     * @brief get median of general vector
     *
     * @param v vector of elements
     * @return median element
     */
    template<typename T>
    T getMedian(std::vector<T>& v)
    {
      std::sort(v.begin(), v.end());
      return getMedianPreSorted(v);
    }

  } // namespace algorithm
} // namespace vision_core

#endif // VISION_CORE_ALGORITMS_HPP
