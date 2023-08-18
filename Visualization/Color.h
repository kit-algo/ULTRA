#pragma once

#include <iostream>
#include <vector>
#include <string>

#include "../Helpers/Assert.h"
#include "../Helpers/String/String.h"

class Color {

public:
    static const Color Black;
    static const Color White;
    static const Color Grey;
    static const Color LightGrey;
    static const Color DarkGrey;
    static const Color KITgreen;
    static const Color KITblue;
    static const Color KITorange;
    static const Color KITred;
    static const Color KITviolet;
    static const Color KITcyan;
    static const Color KITyellow;
    static const Color KITpalegreen;
    static const Color KITseablue;
    static const Color Red;
    static const Color Green;
    static const Color Blue;
    static const Color Cyan;
    static const Color Violet;
    static const Color Yellow;

public:
    Color(const double r = 0.0, const double g = 0.0, const double b = 0.0, const double a = 1.0) :
        r(std::min(1.0, std::max(0.0, r))),
        g(std::min(1.0, std::max(0.0, g))),
        b(std::min(1.0, std::max(0.0, b))),
        a(std::min(1.0, std::max(0.0, a))) {
    }

    Color(const int r, const int g, const int b, const int a = 255) :
        r(std::min(1.0, std::max(0.0, r / 255.0))),
        g(std::min(1.0, std::max(0.0, g / 255.0))),
        b(std::min(1.0, std::max(0.0, b / 255.0))),
        a(std::min(1.0, std::max(0.0, a / 255.0))) {
    }

    Color(const Color& c, const double a = 1.0) :
        r(c.r),
        g(c.g),
        b(c.b),
        a(std::min(1.0, std::max(0.0, a))) {
    }

    Color(const Color c, const int a) :
        r(c.r),
        g(c.g),
        b(c.b),
        a(std::min(1.0, std::max(0.0, a / 255.0))) {
    }

    Color(const std::string& s) :
        Color(byteFromString(s.substr(0, 2)), byteFromString(s.substr(2, 2)), byteFromString(s.substr(4, 2))) {
    }

    template<typename T>
    inline T getR() const noexcept {return r;}
    template<typename T>
    inline T getG() const noexcept {return g;}
    template<typename T>
    inline T getB() const noexcept {return b;}
    template<typename T>
    inline T getA() const noexcept {return a;}

    inline Color operator<<(const double f) const noexcept {
        return Color(r * f, g * f, b * f, a);
    }

    inline Color operator>>(const double f) const noexcept {
        return Color(1 - ((1 - r) * f), 1 - ((1 - g) * f), 1 - ((1 - b) * f), a);
    }

    inline static Color getGradientColor(const Color c1, const Color c2, const double gradient) noexcept {
        const double newR = c1.r + gradient * (c2.r - c1.r);
        const double newG = c1.g + gradient * (c2.g - c1.g);
        const double newB = c1.b + gradient * (c2.b - c1.b);
        const double newA = c1.a + gradient * (c2.a - c1.a);
        return Color(newR, newG, newB, newA);
    }

    inline static Color getGradientColor(const Color c1, const Color c2, const Color c3, const double gradient) noexcept {
        if (gradient < 0.5) {
            return getGradientColor(c1, c2, gradient * 2);
        } else {
            return getGradientColor(c2, c3, (gradient - 0.5) * 2);
        }
    }

private:
    inline std::string byteToString(const int byte) const noexcept {
        std::stringstream ss;
        ss << std::hex << ((byte / 16) % 16) << std::hex << (byte % 16);
        return ss.str();
    }
    inline int byteFromString(const std::string& byte) const noexcept {
        return byteFromChar(byte[0]) * 16 + byteFromChar(byte[1]);
    }
    inline int byteFromChar(const char c) const noexcept {
        if (c >= '0' && c <= '9') return c - '0';
        if (c >= 'a' && c <= 'f') return c - 'a' + 10;
        if (c >= 'A' && c <= 'F') return c - 'A' + 10;
        return 0;
    }

    friend inline std::ostream& operator<<(std::ostream& out, const Color& c) noexcept;

public:
    double r;
    double g;
    double b;
    double a;

};

template<>
inline int Color::getR() const noexcept {return r * 255;}
template<>
inline int Color::getG() const noexcept {return g * 255;}
template<>
inline int Color::getB() const noexcept {return b * 255;}
template<>
inline int Color::getA() const noexcept {return a * 255;}

template<>
inline std::string Color::getR() const noexcept {return byteToString(r * 255);}
template<>
inline std::string Color::getG() const noexcept {return byteToString(g * 255);}
template<>
inline std::string Color::getB() const noexcept {return byteToString(b * 255);}
template<>
inline std::string Color::getA() const noexcept {return byteToString(a * 255);}

inline std::ostream& operator<<(std::ostream& out, const Color& c) noexcept {
    return out << c.getR<std::string>() << c.getG<std::string>() << c.getB<std::string>();
}

const Color Color::Black =        Color(0.000, 0.000, 0.000);
const Color Color::White =        Color(1.000, 1.000, 1.000);
const Color Color::Grey =         Color(0.500, 0.500, 0.500);
const Color Color::LightGrey =    Color(0.750, 0.750, 0.750);
const Color Color::DarkGrey =     Color(0.250, 0.250, 0.250);
const Color Color::KITgreen =     Color(0.000, 0.588, 0.509);
const Color Color::KITblue =      Color(0.274, 0.392, 0.666);
const Color Color::KITorange =    Color(0.862, 0.627, 0.117);
const Color Color::KITred =       Color(0.627, 0.117, 0.156);
const Color Color::KITviolet =    Color(0.627, 0.000, 0.470);
const Color Color::KITcyan =      Color(0.313, 0.666, 0.901);
const Color Color::KITyellow =    Color(0.980, 0.901, 0.078);
const Color Color::KITpalegreen = Color(0.509, 0.745, 0.235);
const Color Color::KITseablue =   Color(0.196, 0.313, 0.549);
const Color Color::Red =          Color(0.800, 0.000, 0.000);
const Color Color::Green =        Color(0.100, 0.780, 0.100);
const Color Color::Blue =         Color(0.000, 0.300, 1.000);
const Color Color::Cyan =         Color(0.200, 0.720, 0.710);
const Color Color::Violet =       Color(1.000, 0.000, 1.000);
const Color Color::Yellow =       Color(1.000, 0.500, 0.000);

const std::vector<Color> colors{Color::KITgreen, Color::KITred, Color::KITblue, Color::KITorange, Color::KITcyan, Color::KITviolet, Color::KITpalegreen, Color::KITseablue};

inline Color cyclicColor(const int i) noexcept {
    return colors[i % colors.size()];
}
