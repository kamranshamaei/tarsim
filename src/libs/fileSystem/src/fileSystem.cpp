/**
 * @file: fileSystem.cpp
 *
 * @Created on: Jul 22, 2017
 * @Author: Kamran Shamaei
 *
 *
 * @brief - Provide file system utilities
 *
 * <Requirement Doc Reference>
 * <Design Doc Reference>
 *
 * @copyright Copyright Kamran Shamaei
 * All Rights Reserved.
 *
 * This file is subject to the terms and conditions defined in
 * file 'LICENSE', which is part of this source code package.
 *
 */

#include "fileSystem.h"
#include <limits.h>

namespace tarsim {
FileSystem::FileSystem()
{

}

FileSystem::~FileSystem()
{

}

bool FileSystem::loadProtoFile(const std::string &fileName, google::protobuf::Message *message)
{
    //Check if file exists
    //return false, if it doesn't exist
    if (!FileSystem::fileExists(fileName))
    {
        std::cout << "Failed to Access the file " << fileName << std::endl;
        return false;
    }

   std::ifstream ifs(fileName);
   google::protobuf::io::IstreamInputStream inputStream(&ifs);

   if (!google::protobuf::TextFormat::Parse(&inputStream, message)) {
      std::cout << "Failed to parse the file " << fileName << std::endl;
      return false;
   }
   return true;
}

std::string FileSystem::getOsName()
{
#ifdef _WIN32
return "Windows 32-bit";
#elif _WIN64
return "Windows 64-bit";
#elif __unix || __unix__
return "Unix";
#elif __APPLE__ || __MACH__
return "Mac OSX";
#elif __linux__
return "Linux";
#elif __FreeBSD__
return "FreeBSD";
#else
return "Other";
#endif
}

bool FileSystem::splitFilename (
        const std::string& fullPath,
        std::string& directory,
        std::string& file)
{
    if (fullPath.size() == 0) {
        return false;
    }

    std::size_t found = -1;
    if (FileSystem::getOsName().find("Windows") != std::string::npos) {
        found = fullPath.find_last_of("/\\");
    } else {
        found = fullPath.find_last_of("/");
    }

    if (found < 0) {
        return false;
    }

    directory = fullPath.substr(0, found + 1);
    file = fullPath.substr(found + 1);
    return true;
}

std::string FileSystem::getexepath()
{
  char result[ PATH_MAX ];
  ssize_t count = readlink( "/proc/self/exe", result, PATH_MAX );
  return std::string( result, (count > 0) ? count : 0 );
}

std::string FileSystem::getexedir()
{
  std::string fullPath = FileSystem::getexepath();
  std::string directory = FileSystem::homeDirectory();
  std::string file_name = "";
  if (!splitFilename (fullPath, directory, file_name)) {
    printf("Failed to get current execution file name, using home\n");
  }
  return directory;
}
} // end of namespace tarsim
