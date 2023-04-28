#pragma once

#include <array>
#include <vector>
#include <string>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <utility>

#include "File.h"

#include "../Assert.h"
#include "../Meta.h"
#include "../Vector/Vector.h"
#include "../FileSystem/FileSystem.h"

namespace IO {

    //################################################# Magic Header ##################################################################//
    inline constexpr int FileHeader = -1;

    //################################################# Type Traits ###################################################################//
    class Serialization;
    class Deserialization;

    namespace ImplementationDetail {
        template<typename T, typename = void>
        struct IsSerializable : Meta::False {};

        template<typename T>
        struct IsSerializable<T, decltype(std::declval<const T>().serialize(std::declval<Serialization&>()), void())> : Meta::True {};

        template<typename T, typename = void>
        struct IsDeserializable : Meta::False {};

        template<typename T>
        struct IsDeserializable<T, decltype(std::declval<T>().deserialize(std::declval<Deserialization&>()), void())> : Meta::True {};

        template<typename T, typename = void>
        struct IsVectorType : Meta::False {};

        template<typename T>
        struct IsVectorType<std::vector<T>, void> : Meta::True {};

        template<typename T, typename = void>
        struct IsArrayType : Meta::False {};

        template<typename T, size_t N>
        struct IsArrayType<std::array<T, N>, void> : Meta::True {};
    }

    template<typename T>
    inline constexpr bool IsSerializable() {return ImplementationDetail::IsSerializable<T>::Value;}

    template<typename T>
    inline constexpr bool IsDeserializable() {return ImplementationDetail::IsDeserializable<T>::Value;}

    template<typename T>
    inline constexpr bool IsVectorType() {return ImplementationDetail::IsVectorType<T>::Value;}

    template<typename T>
    inline constexpr bool IsArrayType() {return ImplementationDetail::IsArrayType<T>::Value;}

    //############################################### Stream Utilities ################################################################//
    template<typename STREAM>
    inline void checkStream(STREAM& stream) {
        Ensure(stream.is_open(), "Cannot access a stream that is not open!");
    }

    template<typename STREAM>
    inline void checkStream(STREAM& stream, const std::string& fileName) {
        Ensure(stream, "cannot open file: " << fileName);
        Ensure(stream.is_open(), "cannot open file: " << fileName);
    }

    //################################################# Serialization #################################################################//
    class Serialization {

    public:
        template<typename... Ts>
        Serialization(const std::string& fileName, const Ts&... objects) :
            fileName(FileSystem::ensureDirectoryExists(fileName)),
            os(fileName, std::ios::binary) {
            checkStream(os, fileName);
            serialize(FileHeader);          // Magic Header signaling that the following data represents a vector serialized by this code
            operator()(objects...);
        }

    public:
        inline void operator()() noexcept {}

        template<typename T, typename... Ts>
        inline void operator()(const T& object, const Ts&... objects) noexcept {
            serialize(object);
            operator()(objects...);
        }

        inline const std::string& getFileName() const noexcept {
            return fileName;
        }

        inline void version(const size_t versionId) noexcept {
            serialize(versionId);
        }

    private:
        template<typename T>
        inline void serialize(const T& object) noexcept {
            checkStream(os);
            if constexpr (IsSerializable<T>()) {
                object.serialize(*this);
            } else {
                os.write(reinterpret_cast<const char*>(&object), sizeof(T));
            }
        }

        inline void serialize(const std::string& stringObject) noexcept {
            checkStream(os);
            serialize(stringObject.size());
            os.write(reinterpret_cast<const char*>(stringObject.data()), stringObject.size());
        }

        template<typename T>
        inline void serialize(const std::vector<T>& vectorObject) noexcept {
            checkStream(os);
            serialize(Meta::type<T>());     // Type of the vector elements, used to check consistency during deserialization
            serialize(vectorObject.size()); // Size of the serialized vector
            if constexpr (Meta::Equals<T, bool>()) {
                serialize(Vector::packBool(vectorObject));
            } else if constexpr (IsSerializable<T>() || IsVectorType<T>() || IsArrayType<T>() || Meta::Equals<T, std::string>()) {
                for (const T& element : vectorObject) {
                    serialize(element);
                }
            } else {
                os.write(reinterpret_cast<const char*>(vectorObject.data()), vectorObject.size() * sizeof(T));
            }
        }

        template<typename T, size_t N>
        inline void serialize(const std::array<T, N>& arrayObject) noexcept {
            checkStream(os);
            serialize(Meta::type<T>());     // Type of the vector elements, used to check consistency during deserialization
            serialize(arrayObject.size());  // Size of the serialized vector
            if constexpr (IsSerializable<T>() || IsVectorType<T>() || IsArrayType<T>() || Meta::Equals<T, std::string>()) {
                for (const T& element : arrayObject) {
                    serialize(element);
                }
            } else {
                os.write(reinterpret_cast<const char*>(arrayObject.data()), arrayObject.size() * sizeof(T));
            }
        }

    private:
        const std::string fileName;
        std::ofstream os;

    };

    //################################################ Deserialization ################################################################//
    class Deserialization {

    public:
        template<typename T, typename... Ts>
        Deserialization(const std::string& fileName, T& object, Ts&... objects) :
            fileName(fileName),
            is(fileName, std::ios::binary) {
            checkStream(is, fileName);
            int header;
            deserialize(header);
            Ensure(header == FileHeader, "No file header found, cannot read the file: " << fileName);
            operator()(object, objects...);
        }
        template<typename T>
        Deserialization(const std::string& fileName, std::vector<T>& object) :
            fileName(fileName),
            is(fileName, std::ios::binary) {
            checkStream(is, fileName);
            int header;
            deserialize(header);
            if (header == FileHeader) {         // Assume that the following vector was serialized using this code
                operator()(object);
            } else {                            // The following data was not serialized using this code
                warning("Trying to deserialize a file (", fileName, ") without magic header!");
                exit(1);
            }
        }

    public:
        inline void operator()() noexcept {}

        template<typename T, typename... Ts>
        inline void operator()(T& object, Ts&... objects) noexcept {
            deserialize(object);
            operator()(objects...);
        }

        inline const std::string& getFileName() const noexcept {
            return fileName;
        }

        inline void version(const size_t expectedVersionId) noexcept {
            size_t fileVersionId = expectedVersionId - 1;
            deserialize(fileVersionId);
            Ensure(fileVersionId == expectedVersionId, "Expected version " << expectedVersionId << ", but file " << fileName << " has version " << fileVersionId);
        }

    private:
        template<typename T>
        inline void deserialize(T& object) noexcept {
            checkStream(is);
            if constexpr (IsDeserializable<T>()) {
                object.deserialize(*this);
            } else {
                is.read(reinterpret_cast<char*>(&object), sizeof(T));
            }
        }

        inline void deserialize(std::string& stringObject) noexcept {
            checkStream(is);
            decltype(stringObject.size()) size = 0;
            deserialize(size);
            stringObject.resize(size);
            is.read(reinterpret_cast<char*>(&stringObject[0]), size);
        }

        template<typename T>
        inline void deserialize(std::vector<T>& vectorObject) noexcept {
            checkStream(is);
            std::string type;
            deserialize(type);
            Ensure(type == Meta::type<T>(), "Trying to deserialize an std::vector<" << Meta::type<T>() << "> from a file that contains an std::vector<" << type << ">!");
            decltype(vectorObject.size()) size = 0;
            deserialize(size);
            if constexpr (Meta::Equals<T, bool>()) {
                std::vector<uint8_t> packedVector;
                deserialize(packedVector);
                vectorObject = Vector::unpackBool(packedVector);
                vectorObject.resize(size);
            } else if constexpr (IsSerializable<T>() || IsVectorType<T>() || IsArrayType<T>() || Meta::Equals<T, std::string>()) {
                vectorObject.resize(size);
                for (T& element : vectorObject) {
                    deserialize(element);
                }
            } else {
                vectorObject.resize(size);
                is.read(reinterpret_cast<char*>(vectorObject.data()), size * sizeof(T));
            }
        }

        template<typename T, size_t N>
        inline void deserialize(std::array<T, N>& arrayObject) noexcept {
            checkStream(is);
            std::string type;
            deserialize(type);
            Ensure(type == Meta::type<T>(), "Trying to deserialize an std::array<" << Meta::type<T>() << "> from a file that contains an std::array<" << type << ">!");
            decltype(arrayObject.size()) size = 0;
            deserialize(size);
            Ensure(size == N, "Trying to deserialize an array of size " << N << " from a file that contains an array of size " << size << "!");
            if constexpr (IsSerializable<T>() || IsVectorType<T>() || IsArrayType<T>() || Meta::Equals<T, std::string>()) {
                for (T& element : arrayObject) {
                    deserialize(element);
                }
            } else {
                is.read(reinterpret_cast<char*>(arrayObject.data()), N * sizeof(T));
            }
        }

    private:
        const std::string fileName;
        std::ifstream is;

    };

    //################################################ File Utilities #################################################################//
    template<typename... Ts>
    inline void serialize(const std::string& fileName, const Ts&... objects) noexcept {
        Serialization(fileName, objects...);
    }

    template<typename T, typename... Ts>
    inline void deserialize(const std::string& fileName, T& object, Ts&... objects) noexcept {
        Deserialization(fileName, object, objects...);
    }

    template<typename T>
    inline T deserialize(const std::string& fileName) noexcept {
        T object;
        deserialize(fileName, object);
        return object;
    }

    template<typename T>
    inline bool checkHeader(const std::string& fileName) noexcept {
        std::ifstream is(fileName, std::ios::binary);
        checkStream(is, fileName);
        int header;
        is.read(reinterpret_cast<char*>(&header), sizeof(int));
        return header == FileHeader;
    }

}
