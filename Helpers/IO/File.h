#pragma once

#include <vector>
#include <string>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <stdio.h>

#include "../Assert.h"
#include "../String/Enumeration.h"
#include "../FileSystem/FileSystem.h"

namespace IO {

    //#################################################### File #####################################################################//
    inline FILE* openFile(const std::string& fileName) noexcept {
        FILE* file = std::fopen(fileName.c_str(), "rb");
        Ensure(file != nullptr, "cannot open file: " << fileName);
        return file;
    }

    inline FILE* openFile(const std::vector<std::string>& fileNameAliases) noexcept {
        FILE* file;
        for (const std::string& fileName : fileNameAliases) {
            file = std::fopen(fileName.c_str(), "rb");
            if (file != nullptr) {
                break;
            }
        }
        if (file == nullptr) {
            Enumeration e;
            for (const std::string& fileName : fileNameAliases) {
                e << fileName << sep;
            }
            Ensure(false, "Cannot open any of the files: " << e);
        }
        return file;
    }

    //################################################## IFStream ###################################################################//
    class IFStream {

    public:
        IFStream(const IFStream& other) = delete;
        IFStream& operator=(const IFStream& other) = delete;

        IFStream(const std::string& fileName, std::ios::openmode mode = std::ios::in) : stream(nullptr) {
            stream = new std::ifstream(fileName, mode);
            Ensure(stream->is_open(), "cannot open file: " << fileName);
        }
        IFStream(const std::vector<std::string>& fileNameAliases, std::ios::openmode mode = std::ios::in) : stream(nullptr) {
            for (const std::string& fileName : fileNameAliases) {
                stream = new std::ifstream(fileName, mode);
                if (stream->is_open()) {
                    break;
                } else {
                    delete stream;
                    stream = nullptr;
                }
            }
            if (stream == nullptr) {
                Enumeration e;
                for (const std::string& fileName : fileNameAliases) {
                    e << fileName << sep;
                }
                Ensure(false, "Cannot open any of the files: " << e);
            }
        }
        IFStream(IFStream&& other) : stream(nullptr) {
            stream = other.stream;
            other.stream = nullptr;
        }

        ~IFStream() {
            close();
        }

        IFStream& operator=(IFStream&& other) noexcept {
            delete stream;
            stream = other.stream;
            other.stream = nullptr;
            return *this;
        }

        inline std::ifstream& getStream() noexcept {
            return *stream;
        }
        inline operator std::ifstream&() noexcept {
            return *stream;
        }

        inline void close() noexcept {
            stream->close();
            delete stream;
            stream = nullptr;
        }
        inline void read(char* s, std::streamsize n) noexcept {
            stream->read(s, n);
        }

    private:
        std::ifstream* stream;

    };

    //################################################## OFStream ###################################################################//
    class OFStream {

    public:
        OFStream(const OFStream& other) = delete;
        OFStream& operator=(const OFStream& other) = delete;

        OFStream(const std::string& fileName, std::ios::openmode mode = std::ios::out) : stream(nullptr) {
            std::string fullFileName = FileSystem::ensureDirectoryExists(fileName);
            stream = new std::ofstream(fullFileName, mode);
            Ensure(stream->is_open(), "cannot open file: " << fullFileName);
            (*stream) << std::setprecision(10);
        }
        OFStream(const std::vector<std::string>& fileNameAliases, std::ios::openmode mode = std::ios::out) : stream(nullptr) {
            for (const std::string& fileName : fileNameAliases) {
                stream = new std::ofstream(fileName, mode);
                if (stream->is_open()) {
                    break;
                } else {
                    delete stream;
                    stream = nullptr;
                }
            }
            if (stream == nullptr) {
                Enumeration e;
                for (const std::string& fileName : fileNameAliases) {
                    e << fileName << sep;
                }
                Ensure(false, "Cannot open any of the files: " << e);
            } else {
                (*stream) << std::setprecision(10);
            }
        }
        OFStream(OFStream&& other) : stream(nullptr) {
            stream = other.stream;
            other.stream = nullptr;
        }

        ~OFStream() {
            close();
        }

        OFStream& operator=(OFStream&& other) noexcept {
            delete stream;
            stream = other.stream;
            other.stream = nullptr;
            return *this;
        }

        template<typename T>
        OFStream& operator<<(const T& t) noexcept {
            (*stream) << t;
            return *this;
        }

        inline std::ofstream& getStream() noexcept {
            return *stream;
        }
        inline operator std::ofstream&() noexcept {
            return *stream;
        }

        inline void flush() noexcept {
            stream->flush();
        }

        inline void close() noexcept {
            stream->flush();
            stream->close();
            delete stream;
            stream = nullptr;
        }
        inline void write(const char* s, std::streamsize n) noexcept {
            stream->write(s, n);
        }

    private:
        std::ofstream* stream;

    };

}
