#pragma once

#include <vector>
#include <array>
#include <string>
#include <cstring>
#include <algorithm>
#include <utility>
#include <cstdio>
#include <exception>
#include <future>
#include <cassert>
#include <cerrno>

#include "File.h"
#include "../Timer.h"
#include "../TaggedInteger.h"
#include "../Vector/Vector.h"
#include "../FileSystem/FileSystem.h"

namespace IO {

namespace Error {

constexpr int MAX_FILE_NAME_LENGTH = 255;

struct Base : std::exception {
    virtual void formatErrorMessage() const = 0;

    const char* what() const throw () {
        formatErrorMessage();
        return errorMessageBuffer;
    }

    mutable char errorMessageBuffer[512];
};

struct WithFileName {
    WithFileName() {std::memset(fileName, 0, MAX_FILE_NAME_LENGTH + 1);}

    void setFileName(const char* fileName) {
        std::strncpy(this->fileName, fileName, MAX_FILE_NAME_LENGTH);
        this->fileName[MAX_FILE_NAME_LENGTH] = '\0';
    }

    char fileName[MAX_FILE_NAME_LENGTH + 1];
};

struct WithFileLine {
    WithFileLine() {fileLine = -1;}

    void setFileLine(int fileLine) {
        this->fileLine = fileLine;
    }

    int fileLine;
};

struct WithErrno {
    WithErrno() {errnoValue = 0;
    }

    void setErrno(int errnoValue) {
        this->errnoValue = errnoValue;
    }

    int errnoValue;
};

struct CanNotOpenFile : Base, WithFileName, WithErrno {
    void formatErrorMessage() const {
        if (errnoValue != 0) {
            std::snprintf(errorMessageBuffer, sizeof(errorMessageBuffer), "Can not open file \"%s\" because \"%s\".", fileName, std::strerror(errnoValue));
        } else {
            std::snprintf(errorMessageBuffer, sizeof(errorMessageBuffer), "Can not open file \"%s\".", fileName);
        }
    }
};

struct LineLengthLimitExceeded : Base, WithFileName, WithFileLine {
    void formatErrorMessage() const {
        std::snprintf(errorMessageBuffer, sizeof(errorMessageBuffer), "Line number %d in file \"%s\" exceeds the maximum length of 2^24-1.", fileLine, fileName);
    }
};

}

class LineReader {

private:
    static constexpr int BLOCK_LENGTH = 1 << 24;
    std::future<int> bytesRead;
    FILE* file;
    char* buffer;
    int dataBegin;
    int dataEnd;

    char fileName[Error::MAX_FILE_NAME_LENGTH + 1];
    unsigned fileLine;

    void init() {
        fileLine = 0;

        // Tell the std library that we want to do the buffering ourself.
        std::setvbuf(file, 0, _IONBF, 0);

        try {
            buffer = new char[3 * BLOCK_LENGTH];
        } catch (...) {
            std::fclose(file);
            throw;
        }

        dataBegin = 0;
        dataEnd = std::fread(buffer, 1, 2 * BLOCK_LENGTH, file);

        // Ignore UTF-8 BOM
        if (dataEnd >= 3 && buffer[0] == '\xEF' && buffer[1] == '\xBB' && buffer[2] == '\xBF') dataBegin = 3;

        if (dataEnd == 2 * BLOCK_LENGTH) {
            bytesRead = std::async(std::launch::async, [=]()->int {return std::fread(buffer + 2 * BLOCK_LENGTH, 1, BLOCK_LENGTH, file);});
        }
    }

public:
    LineReader() = delete;
    LineReader(const LineReader&) = delete;
    LineReader& operator=(const LineReader&) = delete;

    LineReader(const char* fileName, FILE* file) : file(file) {
        setFileName(fileName);
        init();
    }

    LineReader(const std::string& fileName, FILE* file) : file(file) {
        setFileName(fileName.c_str());
        init();
    }

    explicit LineReader(const std::string& fileName) : file(IO::openFile(fileName)) {
        setFileName(fileName.c_str());
        init();
    }

    explicit LineReader(const std::vector<std::string>& fileNameAliases) : file(IO::openFile(fileNameAliases)) {
        setFileName(fileNameAliases.front().c_str());
        init();
    }

    void setFileName(const std::string& fileName) {
        setFileName(fileName.c_str());
    }

    void setFileName(const char* fileName) {
        strncpy(this->fileName, fileName, Error::MAX_FILE_NAME_LENGTH);
        this->fileName[Error::MAX_FILE_NAME_LENGTH] = '\0';
    }

    const char* getTruncatedFileName() const {
        return fileName;
    }

    void setFileLine(unsigned fileLine) {
        this->fileLine = fileLine;
    }

    unsigned getFileLine() const {
        return fileLine;
    }

    char* nextLine() {
        if (dataBegin == dataEnd) return 0;

        fileLine++;

        assert(dataBegin < dataEnd);
        assert(dataEnd <= BLOCK_LENGTH * 2);

        if (dataBegin >= BLOCK_LENGTH) {
            std::memcpy(buffer, buffer + BLOCK_LENGTH, BLOCK_LENGTH);
            dataBegin -= BLOCK_LENGTH;
            dataEnd -= BLOCK_LENGTH;
            if (bytesRead.valid()) {
                dataEnd += bytesRead.get();
                std::memcpy(buffer + BLOCK_LENGTH, buffer + 2 * BLOCK_LENGTH, BLOCK_LENGTH);
                bytesRead = std::async(std::launch::async, [=]()->int {return std::fread(buffer + 2 * BLOCK_LENGTH, 1, BLOCK_LENGTH, file);});
            }
        }

        int lineEnd = dataBegin;
        while (buffer[lineEnd] != '\n' && lineEnd != dataEnd) {
            ++lineEnd;
        }

        if (lineEnd - dataBegin + 1 > BLOCK_LENGTH) {
            Error::LineLengthLimitExceeded error;
            error.setFileName(fileName);
            error.setFileLine(fileLine);
            throw error;
        }

        if (buffer[lineEnd] == '\n') {
            buffer[lineEnd] = '\0';
        } else {
            // some files are missing the newline at the end of the
            // last line
            ++dataEnd;
            buffer[lineEnd] = '\0';
        }

        // handle windows \r\n-line breaks
        if (lineEnd != dataBegin && buffer[lineEnd - 1] == '\r') buffer[lineEnd - 1] = '\0';

        char* ret = buffer + dataBegin;
        dataBegin = lineEnd + 1;
        return ret;
    }

    ~LineReader() {
        if (bytesRead.valid()) bytesRead.get();
        delete[] buffer;
        std::fclose(file);
    }

};


namespace Error {

constexpr int MAX_COLUMN_NAME_LENGTH = 63;
constexpr int MAX_COLUMN_CONTENT_LENGTH = 63;

struct WithColumnName {
    WithColumnName() {std::memset(columnName, 0, MAX_COLUMN_NAME_LENGTH + 1);}

    void setColumnName(const char* columnName) {
        std::strncpy(this->columnName, columnName, MAX_COLUMN_NAME_LENGTH);
        this->columnName[MAX_COLUMN_NAME_LENGTH] = '\0';
    }

    char columnName[MAX_COLUMN_NAME_LENGTH + 1];
};


struct WithColumnContent {
    WithColumnContent() {std::memset(columnContent, 0, MAX_COLUMN_CONTENT_LENGTH + 1);}

    void setColumnContent(const char*columnContent) {
        std::strncpy(this->columnContent, columnContent, MAX_COLUMN_CONTENT_LENGTH);
        this->columnContent[MAX_COLUMN_CONTENT_LENGTH] = '\0';
    }

    char columnContent[MAX_COLUMN_CONTENT_LENGTH + 1];
};

struct ExtraColumnInHeader : Base, WithFileName, WithColumnName {
    void formatErrorMessage() const {
        std::snprintf(errorMessageBuffer, sizeof(errorMessageBuffer), "Extra column \"%s\" in header of file \"%s\".", columnName, fileName);
    }
};

struct MissingColumnInHeader : Base, WithFileName, WithColumnName {
    void formatErrorMessage() const {
        std::snprintf(errorMessageBuffer, sizeof(errorMessageBuffer), "Missing column \"%s\" in header of file \"%s\".", columnName, fileName);
    }
};

struct DuplicatedColumnInHeader: Base, WithFileName, WithColumnName {
    void formatErrorMessage() const {
        std::snprintf(errorMessageBuffer, sizeof(errorMessageBuffer), "Duplicated column \"%s\" in header of file \"%s\".", columnName, fileName);
    }
};

struct HeaderMissing : Base, WithFileName {
    void formatErrorMessage() const {
        std::snprintf(errorMessageBuffer, sizeof(errorMessageBuffer), "Header missing in file \"%s\".", fileName);
    }
};

struct TooFewColumns : Base, WithFileName, WithFileLine {
    void formatErrorMessage() const {
        std::snprintf(errorMessageBuffer, sizeof(errorMessageBuffer), "Too few columns in line %d in file \"%s\".", fileLine, fileName);
    }
};

struct TooManyColumns : Base, WithFileName, WithFileLine {
    void formatErrorMessage() const {
        std::snprintf(errorMessageBuffer, sizeof(errorMessageBuffer), "Too many columns in line %d in file \"%s\".", fileLine, fileName);
    }
};

struct EscapedStringNotClosed : Base, WithFileName, WithFileLine {
    void formatErrorMessage() const {
        std::snprintf(errorMessageBuffer, sizeof(errorMessageBuffer), "Escaped string was not closed in line %d in file \"%s\".", fileLine, fileName);
    }
};

struct IntegerMustBePositive : Base, WithFileName, WithFileLine, WithColumnName, WithColumnContent {
    void formatErrorMessage() const {
        std::snprintf(errorMessageBuffer, sizeof(errorMessageBuffer), "The integer \"%s\" must be positive or 0 in column \"%s\" in file \"%s\" in line \"%d\".", columnContent, columnName, fileName, fileLine);
    }
};

struct NoDigit : Base, WithFileName, WithFileLine, WithColumnName, WithColumnContent {
    void formatErrorMessage() const {
        std::snprintf(errorMessageBuffer, sizeof(errorMessageBuffer), "The integer \"%s\" contains an invalid digit in column \"%s\" in file \"%s\" in line \"%d\".", columnContent, columnName, fileName, fileLine);
    }
};

struct IntegerOverflow : Base, WithFileName, WithFileLine, WithColumnName, WithColumnContent {
    void formatErrorMessage() const {
        std::snprintf(errorMessageBuffer, sizeof(errorMessageBuffer), "The integer \"%s\" overflows in column \"%s\" in file \"%s\" in line \"%d\".", columnContent, columnName, fileName, fileLine);
    }
};

struct IntegerUnderflow : Base, WithFileName, WithFileLine, WithColumnName, WithColumnContent {
    void formatErrorMessage() const {
        std::snprintf(errorMessageBuffer, sizeof(errorMessageBuffer), "The integer \"%s\" underflows in column \"%s\" in file \"%s\" in line \"%d\".", columnContent, columnName, fileName, fileLine);
    }
};

struct InvalidSingleCharacter : Base, WithFileName, WithFileLine, WithColumnName, WithColumnContent {
    void formatErrorMessage() const {
        std::snprintf(errorMessageBuffer, sizeof(errorMessageBuffer), "The content \"%s\" of column \"%s\" in file \"%s\" in line \"%d\" is not a single character.", columnContent, columnName, fileName, fileLine);
    }
};

}

using IgnoreColumn = unsigned;
static constexpr IgnoreColumn IGNORE_NO_COLUMN = 0;
static constexpr IgnoreColumn IGNORE_EXTRA_COLUMN = 1;
static constexpr IgnoreColumn IGNORE_MISSING_COLUMN = 2;

template<char... TRIM_CHAR_LIST>
struct TrimChars {

private:
    constexpr static bool isTrimChar(const char) {
        return false;
    }

    template<class ...OtherTrimChars>
    constexpr static bool isTrimChar(const char c, const char trimChar, OtherTrimChars ...otherTrimChars) {
        return (c == trimChar) || isTrimChar(c, otherTrimChars...);
    }

public:
    static void trim(char*& strBegin, char*& strEnd) {
        while (isTrimChar(*strBegin, TRIM_CHAR_LIST...) && strBegin != strEnd) {
            ++strBegin;
        }
        while (isTrimChar(*(strEnd - 1), TRIM_CHAR_LIST...) && strBegin != strEnd) {
            --strEnd;
        }
        *strEnd = '\0';
    }

};

struct NoComment {
    static bool isComment(const char*) {
        return false;
    }
};

template<char... COMMENT_START_CHAR_LIST>
struct SingleLineComment {

private:
    constexpr static bool isCommentStartChar(const char) {
        return false;
    }

    template<class ...OtherCommentStartChars>
    constexpr static bool isCommentStartChar(const char c, const char commentStartChar, OtherCommentStartChars ...otherCommentStartChars) {
        return c == commentStartChar || isCommentStartChar(c, otherCommentStartChars...);
    }

public:
    static bool isComment(const char* line) {
        return isCommentStartChar(*line, COMMENT_START_CHAR_LIST...);
    }

};

struct EmptyLineComment {
    static bool isComment(const char* line) {
        if (*line == '\0') return true;
        while (*line == ' ' || *line == '\t') {
            ++line;
            if (*line == 0) return true;
        }
        return false;
    }
};

template<char ... COMMENT_START_CHAR_LIST>
struct SingleAndEmptyLineComment {
    static bool isComment(const char* line) {
        return SingleLineComment<COMMENT_START_CHAR_LIST...>::isComment(line) || EmptyLineComment::isComment(line);
    }
};

template<char SEP>
struct NoQuoteEscape {
    constexpr static char Sep = SEP;

    static const char* findNextColumnEnd(const char* colBegin) {
        while (*colBegin != Sep && *colBegin != '\0') {
            colBegin++;
        }
        return colBegin;
    }

    static void unescape(char*&, char*&) noexcept {}
};

template<char SEP, char QUOTE>
struct DoubleQuoteEscape {
    constexpr static char Sep = SEP;

    static const char* findNextColumnEnd(const char* colBegin) {
        while (*colBegin != Sep && *colBegin != '\0') {
            if (*colBegin != QUOTE) {
                colBegin++;
            } else {
                do {
                    colBegin++;
                    while (*colBegin != QUOTE) {
                        if (*colBegin == '\0') throw Error::EscapedStringNotClosed();
                        colBegin++;
                    }
                    colBegin++;
                } while (*colBegin == QUOTE);
            }
        }
        return colBegin;
    }

    static void unescape(char*& colBegin, char*& colEnd) {
        if (colEnd - colBegin >= 2) {
            if (*colBegin == QUOTE && *(colEnd - 1) == QUOTE) {
                colBegin++;
                colEnd--;
                char*out = colBegin;
                for (char*in = colBegin; in != colEnd; ++in) {
                    if (*in == QUOTE && *(in + 1) == QUOTE) {
                        ++in;
                    }
                    *out = *in;
                    out++;
                }
                colEnd = out;
                *colEnd = '\0';
            }
        }

    }
};

struct ThrowOnOverflow {
    template<class T>
    static void onOverflow(T&) {
        throw Error::IntegerOverflow();
    }

    template<class T>
    static void onUnderflow(T&) {
        throw Error::IntegerUnderflow();
    }
};

struct IgnoreOverflow {
    template<class T>
    static void onOverflow(T&) {}

    template<class T>
    static void onUnderflow(T&) {}
};

struct SetToMaxOnOverflow {
    template<class T>
    static void onOverflow(T& x) {
        x = std::numeric_limits<T>::max();
    }

    template<class T>
    static void onUnderflow(T& x) {
        x = std::numeric_limits<T>::min();
    }
};

namespace Detail {

template<class QUOTE_POLICY>
void chopNextColumn(char*& line, char*& colBegin, char*& colEnd) {
    assert(line != nullptr);

    colBegin = line;
    colEnd = colBegin + (QUOTE_POLICY::findNextColumnEnd(colBegin) - colBegin);

    if (*colEnd == '\0') {
        line = nullptr;
    } else {
        *colEnd = '\0';
        line = colEnd + 1;
    }
}

template<class TRIM_POLICY, class QUOTE_POLICY>
std::vector<std::string> parseLine(char* line) {
    std::vector<std::string> rowData;
    while (line) {
        char* colBegin;
        char* colEnd;
        chopNextColumn<QUOTE_POLICY>(line, colBegin, colEnd);

        TRIM_POLICY::trim(colBegin, colEnd);
        QUOTE_POLICY::unescape(colBegin, colEnd);

        rowData.emplace_back(colBegin);
    }
    return rowData;
}

template<class TRIM_POLICY, class QUOTE_POLICY>
void parseLine(char* line, char** sortedCol, const std::vector<int>& colOrder) {
    for (std::size_t i = 0; i < colOrder.size(); ++i) {
        if (line == nullptr) throw ::IO::Error::TooFewColumns();
        char* colBegin;
        char* colEnd;
        chopNextColumn<QUOTE_POLICY>(line, colBegin, colEnd);

        if (colOrder[i] != -1) {
            TRIM_POLICY::trim(colBegin, colEnd);
            QUOTE_POLICY::unescape(colBegin, colEnd);

            sortedCol[colOrder[i]] = colBegin;
        }
    }
    if (line != nullptr) throw ::IO::Error::TooManyColumns();
}

template<unsigned COLUMN_COUNT, class TRIM_POLICY, class QUOTE_POLICY>
void parseHeaderLine(char* line, std::vector<int>& colOrder, const std::array<std::vector<std::string>, COLUMN_COUNT>& columnNameAliases, IgnoreColumn ignorePolicy) {
    colOrder.clear();

    bool found[COLUMN_COUNT];
    std::fill(found, found + COLUMN_COUNT, false);

    while (line) {
        char* colBegin;
        char* colEnd;
        chopNextColumn<QUOTE_POLICY>(line, colBegin, colEnd);

        TRIM_POLICY::trim(colBegin, colEnd);
        QUOTE_POLICY::unescape(colBegin, colEnd);

        for (unsigned i = 0; i < COLUMN_COUNT; ++i) {
            if (Vector::contains(columnNameAliases[i], std::string(colBegin))) {
                if (found[i]) {
                    Error::DuplicatedColumnInHeader error;
                    error.setColumnName(colBegin);
                    throw error;
                }
                found[i] = true;
                colOrder.push_back(i);
                colBegin = 0;
                break;
            }
        }
        if (colBegin) {
            if (ignorePolicy & ::IO::IGNORE_EXTRA_COLUMN) {
                colOrder.push_back(-1);
            } else {
                Error::ExtraColumnInHeader error;
                error.setColumnName(colBegin);
                throw error;
            }
        }
    }
    if (!(ignorePolicy & ::IO::IGNORE_MISSING_COLUMN)) {
        for (unsigned i = 0; i < COLUMN_COUNT; ++i) {
            if (!found[i]) {
                Error::MissingColumnInHeader error;
                error.setColumnName(columnNameAliases[i][0].c_str());
                throw error;
            }
        }
    }
}

template<class OVERFLOW_POLICY>
void parse(const char* col, char& x) {
    if (!*col) throw Error::InvalidSingleCharacter();
    x = *col;
    col++;
    if (*col) throw Error::InvalidSingleCharacter();
}

template<class OVERFLOW_POLICY>
void parse(const char* col, std::string& x) {
    x = col;
}

template<class OVERFLOW_POLICY>
void parse(const char* col, const char*& x) {
    x = col;
}

// template<class OVERFLOW_POLICY>
// void parse(const char* col, char*& x) {
//     x = col;
// }

template<class OVERFLOW_POLICY, class T>
void parseUnsignedInteger(const char* col, T& x) {
    x = 0;
    while (*col != '\0') {
        if ('0' <= *col && *col <= '9') {
            T y = *col - '0';
            if (x > (std::numeric_limits<T>::max() - y) / 10) {
                OVERFLOW_POLICY::onOverflow(x);
                return;
            }
            x = 10 * x + y;
        } else {
            throw Error::NoDigit();
        }
        col++;
    }
}

template<class OVERFLOW_POLICY>
void parse(const char* col, unsigned char& x) {
    parseUnsignedInteger<OVERFLOW_POLICY>(col, x);
}
template<class OVERFLOW_POLICY>
void parse(const char* col, unsigned short& x) {
    parseUnsignedInteger<OVERFLOW_POLICY>(col, x);
}
template<class OVERFLOW_POLICY>
void parse(const char* col, unsigned int& x) {
    parseUnsignedInteger<OVERFLOW_POLICY>(col, x);
}
template<class OVERFLOW_POLICY>
void parse(const char* col, unsigned long& x) {
    parseUnsignedInteger<OVERFLOW_POLICY>(col, x);
}
template<class OVERFLOW_POLICY>
void parse(const char* col, unsigned long long& x) {
    parseUnsignedInteger<OVERFLOW_POLICY>(col, x);
}

template<class OVERFLOW_POLICY, class T>
void parseSignedInteger(const char* col, T& x) {
    if (*col == '-') {
        col++;

        x = 0;
        while (*col != '\0') {
            if ('0' <= *col && *col <= '9') {
                T y = *col - '0';
                if (x < (std::numeric_limits<T>::min() + y) / 10) {
                    OVERFLOW_POLICY::onUnderflow(x);
                    return;
                }
                x = 10 * x - y;
            } else {
                throw Error::NoDigit();
            }
            col++;
        }
        return;
    } else if (*col == '+') {
        col++;
    }
    parseUnsignedInteger<OVERFLOW_POLICY>(col, x);
}

template<class OVERFLOW_POLICY>
void parse(const char* col, signed char& x) {
    parseSignedInteger<OVERFLOW_POLICY>(col, x);
}
template<class OVERFLOW_POLICY>
void parse(const char* col, signed short& x) {
    parseSignedInteger<OVERFLOW_POLICY>(col, x);
}
template<class OVERFLOW_POLICY>
void parse(const char* col, signed int& x) {
    parseSignedInteger<OVERFLOW_POLICY>(col, x);
}
template<class OVERFLOW_POLICY>
void parse(const char* col, signed long& x) {
    parseSignedInteger<OVERFLOW_POLICY>(col, x);
}
template<class OVERFLOW_POLICY>
void parse(const char* col, signed long long& x) {
    parseSignedInteger<OVERFLOW_POLICY>(col, x);
}
template<class OVERFLOW_POLICY>
void parse(const char* col, bool& x) {
    parseUnsignedInteger<OVERFLOW_POLICY>(col, x);
}

template<class OVERFLOW_POLICY, int TAG, typename VALUE_TYPE, VALUE_TYPE INVALID, VALUE_TYPE DEFAULT, typename... ADDITIONAL_CASTS>
void parse(const char* col, TaggedInteger<TAG, VALUE_TYPE, INVALID, DEFAULT, ADDITIONAL_CASTS...>& x) {
    VALUE_TYPE y;
    parseSignedInteger<OVERFLOW_POLICY>(col, y);
    x = TaggedInteger<TAG, VALUE_TYPE, INVALID, DEFAULT, ADDITIONAL_CASTS...>(y);
}

template<class T>
void parseFloat(const char* col, T& x) {
    bool isNegative = false;
    if (*col == '-') {
        isNegative = true;
        col++;
    } else if (*col == '+') {
        col++;
    }

    x = 0;
    if (col[0] == 'n' && col[1] == 'a' && col[2] == 'n' && col[3] == '\0') {
        x = std::numeric_limits<T>::quiet_NaN();
        col += 3;
    } else if (col[0] == 'i' && col[1] == 'n' && col[2] == 'f' && col[3] == '\0') {
        x = std::numeric_limits<T>::infinity();
        col += 3;
    }

    while ('0' <= *col && *col <= '9') {
        int y = *col - '0';
        x *= 10;
        x += y;
        col++;
    }

    if (*col == '.' || *col == ',') {
        col++;
        T pos = 1;
        while ('0' <= *col && *col <= '9') {
            pos /= 10;
            int y = *col - '0';
            col++;
            x += y * pos;
        }
    }

    if (*col == 'e' || *col == 'E') {
        col++;
        int e;

        parseSignedInteger<SetToMaxOnOverflow>(col, e);

        if (e != 0) {
            T base;
            if (e < 0) {
                base = 0.1;
                e = -e;
            } else {
                base = 10;
            }

            while (e != 1) {
                if ((e & 1) == 0) {
                    base = base * base;
                    e >>= 1;
                } else {
                    x *= base;
                    e--;
                }
            }
            x *= base;
        }
    } else {
        if (*col != '\0') throw Error::NoDigit();
    }

    if (isNegative) x = -x;
}

template<class OVERFLOW_POLICY>
void parse(const char* col, float& x) {
    parseFloat(col, x);
}
template<class OVERFLOW_POLICY>
void parse(const char* col, double& x) {
    parseFloat(col, x);
}
template<class OVERFLOW_POLICY>
void parse(const char* col, long double& x) {
    parseFloat(col, x);
}

template<class OVERFLOW_POLICY, class T>
void parse(const char*, T&) {
    static_assert(sizeof(T) != sizeof(T), "Can not parse this type. Only builtin integrals, TaggedInteger, floats, char, char*, const char* and std::string are supported");
}

}

template<unsigned COLUMN_COUNT, class TRIM_POLICY = TrimChars<>, class QUOTE_POLICY = NoQuoteEscape<','>, class OVERFLOW_POLICY = ThrowOnOverflow, class COMMENT_POLICY = EmptyLineComment>
class CSVReader {

private:
    LineReader in;

    char* row[COLUMN_COUNT];

    std::array<std::vector<std::string>, COLUMN_COUNT> columnNameAliases;

    std::vector<int> colOrder;

public:
    CSVReader() = delete;
    CSVReader(const CSVReader&) = delete;
    CSVReader&operator=(const CSVReader&);

    explicit CSVReader(const std::string& fileName) : in(fileName) {
        init();
    }

    explicit CSVReader(const std::vector<std::string>& fileNameAliases) : in(fileNameAliases) {
        init();
    }

    template<typename... T, typename = std::enable_if_t<sizeof...(T) == COLUMN_COUNT>>
    void readHeader(const T&... columnNames) {
        columnNameAliases = std::array<std::vector<std::string>, COLUMN_COUNT>{std::vector<std::string>{columnNames}...};
        readHeader(IGNORE_EXTRA_COLUMN);
    }

    template<typename... T, typename = std::enable_if_t<sizeof...(T) == COLUMN_COUNT>>
    void readHeader(const IgnoreColumn ignorePolicy, const T&... columnNames) {
        columnNameAliases = std::array<std::vector<std::string>, COLUMN_COUNT>{std::vector<std::string>{columnNames}...};
        readHeader(ignorePolicy);
    }

    template<class... COLUMN_TYPE>
    bool readRow(COLUMN_TYPE&... cols){
        static_assert(sizeof...(COLUMN_TYPE) >= COLUMN_COUNT, "not enough columns specified");
        static_assert(sizeof...(COLUMN_TYPE) <= COLUMN_COUNT, "too many columns specified");
        try {
            try {
                char* line;
                do {
                    line = in.nextLine();
                    if(!line) return false;
                } while(COMMENT_POLICY::isComment(line));
                Detail::parseLine<TRIM_POLICY, QUOTE_POLICY>(line, row, colOrder);
                parseHelper(0, cols...);
            } catch(Error::WithFileName& error){
                error.setFileName(in.getTruncatedFileName());
                throw;
            }
        } catch(Error::WithFileLine& error){
            error.setFileLine(in.getFileLine());
            throw;
        }
        return true;
    }

    bool hasColumn(const std::string& name) const {
        int nameIndex = -2;
        for (size_t i = 0; i < COLUMN_COUNT; i++) {
            if (Vector::contains(columnNameAliases[i], name)) {
                nameIndex = i;
                break;
            }
        }
        return Vector::contains(colOrder, nameIndex);
    }

    const char* getTruncatedFileName() const {
        return in.getTruncatedFileName();
    }

    void setFileLine(unsigned fileLine){
        in.setFileLine(fileLine);
    }

    unsigned getFileLine() const {
        return in.getFileLine();
    }

private:
    void init() noexcept {
        std::fill(row, row + COLUMN_COUNT, nullptr);
        colOrder.resize(COLUMN_COUNT);
        for(unsigned i = 0; i < COLUMN_COUNT; i++) {
            colOrder[i] = i;
        }
        for(unsigned i = 1; i <= COLUMN_COUNT; i++) {
            columnNameAliases[i - 1] = std::vector<std::string>(1, "col" + std::to_string(i));
        }
    }

    void parseHelper(std::size_t){}

    template<class T, class... COLUMN_TYPE>
    void parseHelper(std::size_t r, T& t, COLUMN_TYPE&... cols){
        if(row[r]){
            try {
                try {
                    ::IO::Detail::parse<OVERFLOW_POLICY>(row[r], t);
                } catch(Error::WithColumnContent& error){
                    error.setColumnContent(row[r]);
                    throw;
                }
            } catch(Error::WithColumnName& error) {
                error.setColumnName(columnNameAliases[r][0].c_str());
                throw;
            }
        }
        parseHelper(r + 1, cols...);
    }

    void readHeader(IgnoreColumn ignorePolicy) {
        try {
            char* line = nullptr;
            do {
                line = in.nextLine();
                if (!line) throw Error::HeaderMissing();
            } while(COMMENT_POLICY::isComment(line));
            Detail::parseHeaderLine<COLUMN_COUNT, TRIM_POLICY, QUOTE_POLICY>(line, colOrder, columnNameAliases, ignorePolicy);
        } catch(Error::WithFileName& error){
            error.setFileName(in.getTruncatedFileName());
            throw;
        }
    }

};

template<typename PARSE_CONTENT>
inline void readFile(const std::string& fileName, const std::string& contentName, const PARSE_CONTENT& parseContent, const bool verbose = true) {
    if (verbose) std::cout << "Reading " << contentName << " from CSV file (" << fileName << ")..." << std::flush;
    if (FileSystem::isFile(fileName)) {
        Timer timer;
        int count = parseContent();
        if (verbose) std::cout << " done (Found " << String::prettyInt(count) << " " << contentName << " in " << String::msToString(timer.elapsedMilliseconds()) << ")." << std::endl;
    } else {
        if (verbose) std::cout << " file not found." << std::endl;
    }
}

template<typename PARSE_CONTENT>
inline void readFile(const std::vector<std::string>& fileNameAliases, const std::string& contentName, const PARSE_CONTENT& parseContent, const bool verbose = true) {
    if (verbose) std::cout << "Reading " << contentName << " from CSV file (" << fileNameAliases.front() << ")..." << std::flush;
    Timer timer;
    int count = parseContent();
    if (verbose) std::cout << " done (Found " << String::prettyInt(count) << " " << contentName << " in " << String::msToString(timer.elapsedMilliseconds()) << ")." << std::endl;
}

}
