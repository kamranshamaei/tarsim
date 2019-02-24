/**
 *
 * @file: fileSystem.h
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
 *
 */

#ifndef FILE_SYSTEM_H
#define FILE_SYSTEM_H
#include <string>
#include <unistd.h>
#include <fstream>
#include <google/protobuf/text_format.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <iostream>
#include <sys/types.h>
#include <sys/stat.h>
#include <pwd.h>

namespace tarsim {
class FileSystem
{
public:
    FileSystem();
    virtual ~FileSystem();

    static bool fileExists(const std::string &filename)
    {
    	return access( filename.c_str(), F_OK ) != -1;
    }

    static int32_t getPid()
    {
    	return ::getpid() ;
    }
    static std::string getMQNamePid()
    {
        return ("Tarsim" + std::to_string(FileSystem::getPid()));
    }
    static std::string getMQNamePid(int pid)
    {
        return ("Tarsim" + std::to_string(pid));
    }
    static bool pathExists(const std::string &path)
    {
        struct stat s;
        if (stat(path.c_str(), &s) == 0) {
            if (s.st_mode & S_IFDIR) {
                // it's a directory
                return 1;
            } else if (s.st_mode & S_IFREG) {
                // it's a file
                return 2;
            } else {
                // something else
                return -1;
            }
        } else {
            // no file
            return 0;
        }
    }

    static std::string homeDirectory()
    {
        struct passwd *pw = getpwuid(getuid());
        return std::string(pw->pw_dir);
    }

    static bool mkDir(const std::string &dirName)
    {
        if (mkdir(dirName.c_str(), 0777) != 0) {
            return false;
        }
        return true;
    }

    static void recursiveMkDir(const char *dir)
    {
        char tmp[256];
        char *p = NULL;
        size_t len;

        snprintf(tmp, sizeof(tmp),"%s",dir);
        len = strlen(tmp);
        if(tmp[len - 1] == '/')
                tmp[len - 1] = 0;
        for(p = tmp + 1; *p; p++)
                if(*p == '/') {
                        *p = 0;
                        mkdir(tmp, S_IRWXU);
                        *p = '/';
                }
        mkdir(tmp, S_IRWXU);
    }


    static bool loadProtoFile(const std::string &fileName, google::protobuf::Message *message);
    static std::string getOsName();
    static bool splitFilename(
            const std::string& fullPath,
            std::string& directory,
            std::string& file);

    static std::string getexepath();
    static std::string getexedir();
};
}; // end of namespace tarsim
#endif /* FILE_SYSTEM_H */

