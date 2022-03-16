#pragma once

#include <cmath>
#include <vector>
#include <ostream>
#include <sstream>
#include <limits>

#include "Point.h"

#include "../../Helpers/Assert.h"
#include "../../Helpers/Helpers.h"

namespace Geometry {

class Rectangle {

public:
    // Constructors
    Rectangle() : min(), max() {}
    Rectangle(const Point& p) : min(p), max(p) {}
    inline static Rectangle BoundingBox(const Point& a, const Point& b) {
        Rectangle r;
        r.min = Geometry::min(a, b);
        r.max = Geometry::max(a, b);
        Assert(r.isPositive());
        return r;
    }
    template<typename T>
    inline static Rectangle BoundingBox(const std::vector<T>& points) {
        Rectangle r = Empty();
        r.extend(points);
        return r;
    }
    inline static Rectangle MinMax(const Point& min, const Point& max) {
        Rectangle r;
        r.min = min;
        r.max = Geometry::max(min, max);
        Assert(r.isPositive());
        return r;
    }
    inline static Rectangle Union(const Rectangle& a, const Rectangle& b) {
        Rectangle r;
        r.min = Geometry::min(a.min, b.min);
        r.max = Geometry::max(a.max, b.max);
        Assert(r.isPositive());
        return r;
    }
    template<typename T>
    inline static Rectangle Union(const std::vector<T>& rectangles) {
        Rectangle r = Empty();
        r.extend(rectangles);
        return r;
    }
    inline static Rectangle Intersection(const Rectangle& a, const Rectangle& b) {
        Rectangle r;
        r.min = Geometry::max(a.min, b.min);
        r.max = Geometry::min(a.max, b.max);
        r.max.max(r.min);
        Assert(r.isPositive());
        return r;
    }
    inline static Rectangle Negative(const double d = 1000000) {
        Rectangle r;
        r.min = Point(Construct::XY, d, d);
        r.max = Point(Construct::XY, -d, -d);
        return r;
    }
    inline static Rectangle Empty() {
        return Negative(std::numeric_limits<int>::max());
    }

    // Modifiers & Operators
    inline bool operator==(const Rectangle& r) const noexcept {
        return min == r.min && max == r.max;
    }

    inline bool operator!=(const Rectangle &r) const noexcept {
        return !operator==(r);
    }

    inline void extend(const Point& p) noexcept {
        min.min(p);
        max.max(p);
        Assert(isPositive());
    }

    inline void extend(const Rectangle& r) noexcept {
        min.min(r.min);
        max.max(r.max);
        Assert(isPositive());
    }

    template<typename T>
    inline void extend(const std::vector<T>& points) noexcept {
        for (const T& t : points) {
            extend(t);
        }
    }

    inline bool contains(const Point& p) const noexcept {
        if (p.x > max.x || p.x < min.x) return false;
        if (p.y > max.y || p.y < min.y) return false;
        return true;
    }

    inline bool contains(const Rectangle& r) const noexcept {
        if (r.max.x > max.x || r.min.x < min.x) return false;
        if (r.max.y > max.y || r.min.y < min.y) return false;
        return true;
    }

    inline bool intersects(const Rectangle& r) const noexcept {
        return Intersection(*this, r).isPositive();
    }

    inline void discretize(const double dx, const double dy) noexcept {
        min.x = floor(min.x, dx);
        min.y = floor(min.y, dy);
        max.x = ceil(max.x, dx);
        max.y = ceil(max.y, dy);
    }

    inline void clear(const Point& p = Point()) noexcept {
        min = p;
        max = p;
    }

    inline Point center() const noexcept {
        return 0.5 * (min + max);
    }

    inline double area() const noexcept {
        Assert(isPositive());
        return dx() * dy();
    }

    inline Point closestPoint(const Point& p) const noexcept {
        Point c = p;
        if (p.x < min.x) {
            c.x = min.x;
        } else if (p.x > max.x) {
            c.x = max.x;
        }
        if (p.y < min.y) {
            c.y = min.y;
        } else if (p.y > max.y) {
            c.y = max.y;
        }
        return c;
    }

    inline bool isPositive() const noexcept {
        return (min.x <= max.x) || (min.y <= max.y);
    }

    inline bool isPoint() const noexcept {
        return min == max;
    }

    // String conversion
    inline friend std::ostream& operator<<(std::ostream& os, const Rectangle& r) noexcept {
        std::stringstream ss;
        ss << "[" << r.min << " | " << r.max << "]";
        return os << ss.str();
    }

    // Access
    inline double dx() const noexcept {return max.x - min.x;}
    inline double dy() const noexcept {return max.y - min.y;}
    inline double d(const int dimension) const noexcept {return max[dimension] - min[dimension];}

public:
    union {
        Point topLeft;
        Point min;
    };
    union {
        Point bottomRight;
        Point max;
    };

};

// Free functions
inline double euclideanDistanceSquared(const Point& a, const Rectangle& b) noexcept {
    return euclideanDistanceSquared(a, b.closestPoint(a));
}

inline double euclideanDistance(const Point& a, const Rectangle& b) noexcept {
    return std::sqrt(euclideanDistanceSquared(a, b));
}

static_assert(sizeof(Rectangle) == 2 * sizeof(Point), "Rectangle layout is broken");

}
