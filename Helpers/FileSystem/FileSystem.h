#pragma once

#include <algorithm>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>

#include <termios.h>
#include <unistd.h>
#include <stdio.h>
#include <limits.h>

#include <dirent.h>

#include <sys/stat.h>
#include <sys/wait.h>
#include <sys/types.h>

#include "../Assert.h"
#include "../String/String.h"

namespace FileSystem {

    inline std::string getWorkingDirectory() noexcept {
        char buff[PATH_MAX];
        ssize_t len = readlink("/proc/self/exe", buff, sizeof(buff)-1);
        if (len != -1) {
            buff[len] = '\0';
            std::string path(buff);
            return path.substr(0, path.rfind('/'));
        }
        return "/home";
    }

    inline std::vector<std::string> getFiles(const std::string& path) noexcept {
        DIR* dir;
        struct dirent* ent;
        if ((dir = opendir(path.c_str())) != NULL) {
            std::vector<std::string> dirs;
            while ((ent = readdir(dir)) != NULL) {
                dirs.push_back(ent->d_name);
            }
            closedir(dir);
            std::sort(dirs.begin(), dirs.end());
            return dirs;
        }
        return std::vector<std::string>();
    }

    inline bool isAbsolutePath(const std::string& path) noexcept {
        return (!path.empty()) && (path[0] == '/');
    }

    inline bool isDirectory(const std::string& path) noexcept {
        DIR* dir;
        if ((dir = opendir(path.c_str())) != NULL) {
            closedir(dir);
            return true;
        } else {
            return false;
        }
    }

    inline bool isFile(const std::string& path) noexcept {
        std::ifstream f(path.c_str());
        if (f.good()) {
            f.close();
            return true;
        } else {
            return false;
        }
    }

    inline bool renameFile(const std::string& oldName, const std::string& newName) noexcept {
        if (!isFile(oldName)) return false;
        if (isFile(newName)) return false;
        return rename(oldName.c_str(), newName.c_str()) == 0;
    }

    inline void deleteFile(const std::string& fileName) noexcept {
        if (!isFile(fileName)) return;
        remove(fileName.c_str());
    }

    inline bool copyFile(const std::string& oldName, const std::string& newName) noexcept {
        if (!isFile(oldName)) return false;
        if (isFile(newName)) return false;
        std::ifstream source(oldName, std::ios::binary);
        AssertMsg(source.is_open(), "cannot open file: " << oldName);
        std::ofstream destination(newName, std::ios::binary);
        AssertMsg(destination.is_open(), "cannot open file: " << newName);
        destination << source.rdbuf();
        source.close();
        destination.close();
        return true;
    }

    inline bool isFileOrDirectory(const std::string& path) noexcept {
        return isFile(path) || isDirectory(path);
    }

    inline std::string getParentDirectory(const std::string& fileName) noexcept {
        if (fileName.size() < 2) return "";
        size_t directoryEnd = fileName.find_last_of('/', fileName.size() - 2);
        if (directoryEnd >= fileName.size()) return "";
        return fileName.substr(0, directoryEnd);
    }

    inline std::string extendPath(const std::string& base, const std::string& file) noexcept {
        if (file.size() < 1) {
            return base;
        } else if (isAbsolutePath(file) || base.empty()) {
            return file;
        } else if (file[0] == '.') {
            if (file.size() < 2) {
                return base;
            } else if (file[1] == '/') {
                return extendPath(base, file.substr(2));
            }
            AssertMsg(file[1] == '.', "Cannot extend path '" << base << "' with file '" << file << "'!");
            if (file.size() == 2) {
                return getParentDirectory(base);
            }
            AssertMsg(file[2] == '/', "Cannot extend path '" << base << "' with file '" << file << "'!");
            return extendPath(getParentDirectory(base), file.substr(3));
        }
        if (base[base.length() - 1] == '/') {
            return base + file;
        } else {
            return base + "/" + file;
        }
    }

    inline std::string getAbsolutePath(const std::string& path) noexcept {
        if (isAbsolutePath(path)) {
            return path;
        } else {
            return extendPath(getWorkingDirectory(), path);
        }
    }

    inline std::string ensureExtension(const std::string& fileName, const std::string& extension) noexcept {
        if (String::endsWith(fileName, extension)) {
            return fileName;
        } else {
            return fileName + extension;
        }
    }

    inline std::string getFileNameWithoutExtension(const std::string& fileName) noexcept {
        size_t directoryEnd = fileName.find_last_of('/') + 1;
        if (directoryEnd >= fileName.size()) directoryEnd = 0;
        size_t extensionBegin  = fileName.find_last_of('.');
        if (extensionBegin <= directoryEnd) extensionBegin = fileName.size();
        return fileName.substr(directoryEnd, extensionBegin - directoryEnd);
    }

    inline void unzip(const std::string& zipFileName, const std::string& ouputDirectory, const bool verbose = true, const std::string& unzipExecutable = "/usr/bin/unzip") noexcept {
        if (!isFile(zipFileName)) return;
        if (!isDirectory(ouputDirectory)) return;
        pid_t pid = fork();
        switch (pid) {
        case -1:
            error("Failure during fork()!");
            exit(1);
        case 0:
            if (verbose) {
                execl(unzipExecutable.c_str(), "unzip", "-o", zipFileName.c_str(), "-d", ouputDirectory.c_str(), nullptr);
            } else {
                execl(unzipExecutable.c_str(), "unzip", "-o", "-qq", zipFileName.c_str(), "-d", ouputDirectory.c_str(), nullptr);
            }
            error(unzipExecutable + " failed!");
            exit(1);
        default:
            int status = 0;
            while (!WIFEXITED(status)) {
                waitpid(pid, &status, 0);
            }
        }
    }

    inline void makeDirectory(const std::string& path) noexcept {
        if (path.empty()) return;
        if (isDirectory(path)) return;
        makeDirectory(getParentDirectory(path));
        mkdir(path.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    }

    inline const std::string& ensureDirectoryExists(const std::string& fileName) noexcept {
        const std::string parentDirectory = getParentDirectory(fileName);
        if (parentDirectory == "") return fileName;
        if (isDirectory(parentDirectory)) return fileName;
        makeDirectory(parentDirectory);
        return fileName;
    }

}
