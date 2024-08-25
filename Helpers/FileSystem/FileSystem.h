#pragma once

#include <algorithm>
#include <iostream>
#include <filesystem>
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
        std::vector<std::string> dirs;
        for (const auto& entry : std::filesystem::directory_iterator(path)) {
            dirs.emplace_back(entry.path().filename());
        }
        return dirs;
    }

    inline bool isAbsolutePath(const std::string& path) noexcept {
        return (!path.empty()) && (path[0] == '/');
    }

    inline bool isDirectory(const std::string& path) noexcept {
        return std::filesystem::is_directory(path);
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

    inline std::string getParentDirectory(const std::string& fileName) noexcept {
        return std::filesystem::path(fileName).parent_path();
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
            Assert(file[1] == '.', "Cannot extend path '" << base << "' with file '" << file << "'!");
            if (file.size() == 2) {
                return getParentDirectory(base);
            }
            Assert(file[2] == '/', "Cannot extend path '" << base << "' with file '" << file << "'!");
            return extendPath(getParentDirectory(base), file.substr(3));
        }
        if (base[base.length() - 1] == '/') {
            return base + file;
        } else {
            return base + "/" + file;
        }
    }

    inline const std::string& ensureDirectoryExists(const std::string& fileName) noexcept {
        std::filesystem::create_directories(getParentDirectory(fileName));
        return fileName;
    }

}
