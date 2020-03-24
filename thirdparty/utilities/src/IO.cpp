#include <utilities/IO.hpp>

namespace utilities
{

  std::vector<std::string> IO::getLinesFromFile(const std::string& file_name)
  {
    std::vector<std::string> vgmf_files;

    std::ifstream file(file_name);

    if(file.is_open())
    {
      std::string s;

      while(std::getline(file, s))
      {
        vgmf_files.push_back(s);
      }
    }
    else
    {
      std::cout << "unable to open file: " << file_name;
    }

    return vgmf_files;
  }

  void IO::getAbsPathsFromFile(const std::string& list_file, IO::vTokens& vTokens)
  {
    if(!list_file.empty())
    {
      IO::vTokens v        = IO::getLinesFromFile(list_file);
      std::string base_dir = IO::getFileDir(list_file);

      for(std::string& x : v)
      {
        if(!IO::isAbsolutePath(x))
        {
          x = std::string(base_dir + "/" + x);
        }

        vTokens.push_back(x);
      }
    }
  }

}
