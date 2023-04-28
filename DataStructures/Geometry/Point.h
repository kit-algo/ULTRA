#pragma once

#include <cmath>
#include <ostream>
#include <sstream>

#include "../../Helpers/Helpers.h"
#include "../../Helpers/Types.h"
#include "../../Helpers/ConstructorTags.h"

namespace Geometry {

class Point {

public:
    // Constructors
    Point() : latitude(0.0), longitude(0.0) {}
    Point(const Construct::XYTag, const double& x, const double& y) : y(y), x(x) {}
    Point(const Construct::LatLongTag, const double& latitude, const double& longitude) : latitude(latitude), longitude(longitude) {}

    // Modifiers & Operators
    inline void min(const Point& p) {
        x = std::min(x, p.x);
        y = std::min(y, p.y);
    }

    inline void max(const Point& p) {
        x = std::max(x, p.x);
        y = std::max(y, p.y);
    }

    inline Point& operator+=(Point& p) {
        x += p.x;
        y += p.y;
        return *this;
    }

    inline Point& operator-=(Point& p) {
        x -= p.x;
        y -= p.y;
        return *this;
    }

    inline Point& operator*=(const double scalar) {
        x *= scalar;
        y *= scalar;
        return *this;
    }

    inline Point& operator/=(const double divisor) {
        x /= divisor;
        y /= divisor;
        return *this;
    }

    inline bool operator==(const Point& p) const {
        return (x == p.x) && (y == p.y);
    }

    inline bool operator!=(const Point& p) const {
        return !operator==(p);
    }

    inline double abs() const {
        return std::sqrt((x * x) + (y * y));
    }

    inline double absSquared() const {
        return (x * x) + (y * y);
    }

    inline friend Point operator*(const double scalar, const Point& point) {
        return Point(Construct::XY, point.x * scalar, point.y * scalar);
    }

    inline friend Point operator*(const Point& point, const double scalar) {
        return Point(Construct::XY, scalar * point.x, scalar * point.y);
    }

    inline friend double operator*(const Point& a, const Point& b) {
        return (a.x * b.x) + (a.y * b.y);
    }

    inline friend Point operator/(const Point& point, const double divisor) {
        return Point(Construct::XY, point.x / divisor, point.y / divisor);
    }

    inline friend Point operator+(const Point& a, const Point& b) {
        return Point(Construct::XY, a.x + b.x, a.y + b.y);
    }

    inline friend Point operator-(const Point& a, const Point& b) {
        return Point(Construct::XY, a.x - b.x, a.y - b.y);
    }

    inline friend double dotProduct(const Point& a, const Point& b) {
        return (a.x * b.x) + (a.y * b.y);
    }

    inline double distanceToPoint(const Point& p) const {
        const double dx = x - p.x;
        const double dy = y - p.y;
        return sqrt((dx * dx) + (dy * dy));
    }

    inline double distanceToLine(const Point& p, const Point& q) const {
        const Point direction = q - p;
        const Point closestPointOnLine = p + (dotProduct(*this - p, direction) * (direction / direction.absSquared()));
        return distanceToPoint(closestPointOnLine);
    }

    // String conversion
    inline std::string toXY() const {
        std::stringstream ss;
        ss << "\"X\":" << x << ", \"Y\":" << y;
        return ss.str();
    }

    inline std::string toLatLong() const {
        std::stringstream ss;
        ss << "\"Lat\":" << latitude << ", \"Lon\":" << longitude;
        return ss.str();
    }

    inline std::ostream& writeXY(std::ostream& stream) const {
        return stream << toXY();
    }

    inline std::ostream& writeLatLong(std::ostream& stream) const {
        return stream << toLatLong();
    }

    inline friend std::ostream& operator<<(std::ostream& os, const Point& p) {
        std::stringstream ss;
        ss << "(" << p.x << ", " << p.y << ")";
        return os << ss.str();
    }

    // Access
    inline double& operator[](const int dimension) {return (&latitude)[dimension];}
    inline const double& operator[](const int dimension) const {return (&latitude)[dimension];}

public:
    union {
        double latitude;
        double y;
    };
    union {
        double longitude;
        double x;
    };

};

// Free functions
inline Point min(const Point& a, const Point& b) {
    return Point(Construct::XY, std::min(a.x, b.x), std::min(a.y, b.y));
}

inline Point max(const Point& a, const Point& b) {
    return Point(Construct::XY, std::max(a.x, b.x), std::max(a.y, b.y));
}

inline double geoDistanceInCM(const Point& from, const Point& to) {
    if (from == to) return 0;
    double heightFrom(degreesToRadians(from.longitude));
    double heightTo(degreesToRadians(to.longitude));
    double widthFrom(degreesToRadians(from.latitude));
    double widthTo(degreesToRadians(to.latitude));
    return acos(std::min(1.0, sin(widthFrom) * sin(widthTo) + cos(widthFrom) * cos(widthTo) * cos(heightTo - heightFrom))) * EARTH_RADIUS_IN_CENTIMETRE;
}

inline double euclideanDistanceSquared(const Point& a, const Point& b) {
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;
    return (dx * dx) + (dy * dy);
}

inline double euclideanDistance(const Point& a, const Point& b) {
    return std::sqrt(euclideanDistanceSquared(a, b));
}

static_assert(sizeof(Point) == 2 * sizeof(double), "Point layout is broken");

}
