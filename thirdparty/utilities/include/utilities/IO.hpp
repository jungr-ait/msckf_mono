/******************************************************************************
* FILENAME:     IO.hpp
* PURPOSE:      IO
* AUTHOR:       jungr - Roland Jung
* MAIL:         Roland.Jung@ait.ac.at
* VERSION:      v1.0.0
* CREATION:     03.11.2016
*
*  Copyright (C) 2017 AIT Austrian Institute of Technology GmbH
*  All rights reserved. See the LICENSE file for details.
******************************************************************************/
#ifndef UTILS_IO_HPP
#define UTILS_IO_HPP
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <sys/types.h> // required for stat.h
#include <sys/stat.h> // no clue why required -- man pages say so
#include <fts.h>
#include <stdlib.h>
#include <dirent.h>
#include <string.h>
#include <chrono>
#include <iostream>


namespace utilities
{
  namespace IO
  {
    template<typename Map>
    bool key_value_compare(Map const& lhs, Map const& rhs)
    {

      auto pred = [](decltype(*lhs.begin()) a, decltype(a) b)
      {
        return (a.first == b.first) && (a.second == b.second);
      };

      return lhs.size() == rhs.size() && std::equal(lhs.begin(), lhs.end(), rhs.begin(), pred);
    }

    template<class T>
    std::ostream& operator<<(std::ostream& os, const std::vector<T>& v)
    {
      os << "[";

      for(typename std::vector<T>::const_iterator ii = v.begin(); ii != v.end(); ++ii)
      {
        os << " " << *ii;
      }

      os << " ]";
      return os;
    }

    template<typename T2, typename T1>
    inline T2 lexical_cast(const T1& in)
    {
      T2                out;
      std::stringstream ss;
      ss << in;
      ss >> out;
      return out;
    }

    static inline std::string getFirstTokenAndReset(std::ifstream& os, std::string const delim = " ")
    {
      int cur_pos = os.tellg();

      if(cur_pos < 0)
      {
        return std::string();
      }

      std::string t;
      os >> t;
      os.seekg(cur_pos);
      return t;
    }

    typedef std::vector<std::string> vTokens;
    static inline void getTokens(vTokens& tokens, std::string const& str, std::string const delim)
    {
      size_t prev = 0, pos = 0;
      tokens.clear();

      do
      {
        pos = str.find(delim, prev);

        if(pos == std::string::npos || (pos == 0 && prev != 0))
        {
          pos = str.length();
        }

        std::string token = str.substr(prev, pos - prev);

        if(!token.empty())
        {
          tokens.emplace_back(std::move(token));
        }

        prev = pos + delim.length();
      }
      while(pos < str.length() && prev < str.length());
    }

    static inline std::vector<std::string> getTokens(std::string const& str)
    {
      std::stringstream        ss(str);
      std::vector<std::string> Tokens;
      std::string              Temp;

      while(ss >> Temp)
      {
        Tokens.push_back(Temp);
      }

      return Tokens;
    }


    static inline std::string getFirstToken(std::string const& str, std::string const delim = ",")
    {
      size_t prev = 0, pos = 0;
      pos = str.find(delim, prev);

      if(pos == std::string::npos || (pos == 0 && prev != 0))
      {
        return str;
      }

      return str.substr(prev, pos - prev);

    }

    static inline std::vector<std::string> getTokens(std::string const& str, std::string const delim)
    {
      std::vector<std::string> tokens;
      getTokens(tokens, str, delim);
      return tokens;
    }

    static inline bool fileExists(const std::string& name)
    {
      std::ifstream f(name.c_str());
      return f.good();
    }

    static inline bool dirExist(const std::string& dir)
    {
      DIR *d = opendir(dir.c_str());

      if(d != NULL)
      {
        closedir(d);
        return true;
      }

      return false;
    }

    static inline bool isAbsolutePath(const std::string& path)
    {
      return (path.data()[0] == '/');
    }

    static inline std::string getFileExtension(const std::string& filename)
    {
      std::string::size_type idx;

      idx = filename.rfind('.');

      if(idx != std::string::npos)
      {
        return std::string(filename.substr(idx + 1));
      }
      else
      {
        return std::string();
      }
    }

    static inline std::string getFileDir(const std::string& filename)
    {
      std::string::size_type idx;

      idx = filename.rfind('/');

      if(idx != std::string::npos)
      {
        return std::string(filename.substr(0, idx));
      }
      else
      {
        return std::string();
      }
    }

    static inline std::string getFileName(const std::string& filename)
    {
      std::string::size_type idx1, idx2;

      // TODO: this works only for absolute paths

      idx1 = filename.rfind('/');
      idx2 = filename.rfind('.');

      if(idx1 == std::string::npos && idx2 != std::string::npos)
      {
        // no absolute path
        return std::string(filename.substr(0, idx2));
      }
      else if(idx1 != std::string::npos && idx2 != std::string::npos)
      {
        int len = (idx2) - (idx1 + 1);
        return std::string(filename.substr(idx1 + 1, len));
      }
      else
      {
        return std::string();
      }
    }

    static inline std::string getFileParentDirName(const std::string& filename)
    {
      std::string            file_dir = getFileDir(filename);
      std::string::size_type idx      = file_dir.rfind('/');

      if(idx != std::string::npos)
      {
        return std::string(file_dir.substr(idx + 1, file_dir.length()));
      }
      else
      {
        return std::string();
      }
    }


    static inline bool createDirectory(const std::string& dirName)
    {
      std::string sPath  = dirName;
      mode_t      nMode  = 0733; // UNIX style permissions
      int         nError = 0;
#if defined(_WIN32)
      nError = _mkdir(sPath.c_str()); // can be used on Windows
#else
      nError = mkdir(sPath.c_str(), nMode); // can be used on non-Windows
#endif

      if(nError != 0)
      {
        // handle your error here
        return false;
      }

      return true;
    }

    static inline bool createDirectoryRecursive(const std::string& dirName)
    {
      std::vector<std::string> vTokens = getTokens(dirName, "/");

      if(vTokens.size() < 1)
      {
        return false;
      }

      std::string path;

      if(isAbsolutePath(dirName)) // check if it is absolute
      {
        path = "/";
      }

      for(std::string& node : vTokens)
      {
        if(node != "")
        {

          path += node;

          if(!fileExists(path))
          {
            std::cout << "create " << path << std::endl;

            if(!createDirectory(path))
            {
              return false;
            }
          }

          path += "/";
        }
      }

      return true;
    }

    static inline bool removeDirectory(const std::string& dirName)
    {
      int res = system(std::string("rm -r " + dirName).c_str());
      return (res == 0);
    }

    static inline void getFilesInDirectory(std::vector<std::string>& out,
                                           const std::string& directory,
                                           std::string extension = "")
    {
      out.clear();

      if(!dirExist(directory))
      {
        return;
      }

      DIR           *dir;
      struct dirent *ent;
      struct stat   st;

      dir = opendir(directory.c_str());

      bool bCheckExtension = true;

      if(extension.empty())
      {
        bCheckExtension = false;
      }

      while((ent = readdir(dir)) != NULL)
      {
        const std::string file_name      = ent->d_name;
        const std::string full_file_name = directory + "/" + file_name;

        if(file_name[0] == '.')
        {
          continue;
        }

        if(stat(full_file_name.c_str(), &st) == -1)
        {
          continue;
        }

        const bool is_directory = (st.st_mode & S_IFDIR) != 0;

        if(is_directory)
        {
          continue;
        }

        if(bCheckExtension)
        {
          if(getFileExtension(file_name) != extension)
          {
            continue;
          }
        }

        out.push_back(full_file_name);
      }

      closedir(dir);
    } // getFilesInDirectory

    static inline bool openFile(const std::string& filename, std::fstream& f)
    {

      if(!fileExists(filename))
      {
        std::string baseDir = getFileDir(filename);

        if(baseDir.empty())
        {
          return false;
        }

        if(!dirExist(baseDir))
        {
          createDirectoryRecursive(baseDir);

          if(!dirExist(baseDir))
          {
            std::cerr << "something went wrong creating dir: " << baseDir << std::endl;
            return false;
          }
        }
      }

      f.open(filename.c_str(), std::ios_base::out);

      if(!f.is_open())
      {
        return false;
      }

      return true;
    }

    static inline bool createDirectoryFull(const std::string& baseDir)
    {
      if(!dirExist(baseDir))
      {
        createDirectoryRecursive(baseDir);

        if(!dirExist(baseDir))
        {
          std::cerr << "something went wrong creating dir: " << baseDir << std::endl;
          return false;
        }
      }

      return true;
    }

    std::vector<std::string> getLinesFromFile(std::string const& file_name);

    void getAbsPathsFromFile(std::string const& list_file, utilities::IO::vTokens& vTokens);

    static inline std::string checkPathString(std::string pathString)
    {
      // execute the string replacement as long as there is a double slash in the string
      for(; ;)
      {
        int pos = pathString.find("//");

        if(pos < 0)
        {
          break;
        }

        pathString.replace(pos, 2, "/");
      }

      return pathString;
    }

  } // namespace IO
} // namespace utilities


#endif // UTILS_IO_HPP
