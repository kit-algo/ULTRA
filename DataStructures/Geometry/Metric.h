#pragma once

#include <iostream>
#include <vector>
#include <string>
#include <cmath>

#include "Point.h"
#include "Rectangle.h"

namespace Geometry {

struct EuclideanMetric {

    EuclideanMetric() {}

    inline double distanceSquare(const Point& a, const Point& b) const {return Geometry::euclideanDistanceSquared(a, b);}
    inline double distance(const Point& a, const Point& b) const {return Geometry::euclideanDistance(a, b);}
    inline double spread(const Rectangle& r, const int dimension) const {return r.d(dimension);}

};

struct GeoMetric {

    GeoMetric() {}

    inline double distanceSquare(const Point& a, const Point& b) const {
        const double d = geoDistanceInCM(a, b);
        return d * d;
    }

    inline double distance(const Point& a, const Point& b) const {
        return geoDistanceInCM(a, b);
    }

    inline double spread(const Rectangle& r, const int dimension) const {
        Point a = r.center();
        a[dimension] = r.min[dimension];
        Point b = r.center();
        b[dimension] = r.max[dimension];
        return geoDistanceInCM(a, b);
    }

};

struct GeoMetricAproximation {

    GeoMetricAproximation() : correction(Point(Construct::XY, 1, 1)) {}
    inline static GeoMetricAproximation SetCorrection(const Point& c) {
        GeoMetricAproximation metric;
        metric.correction = c;
        return metric;
    }
    inline static GeoMetricAproximation ComputeCorrection(const Point& p) {
        GeoMetricAproximation metric;
        const Point x = Point(Construct::XY, p.x + 0.1, p.y);
        const Point y = Point(Construct::XY, p.x, p.y + 0.1);
        const double dx = geoDistanceInCM(p, x);
        const double dy = geoDistanceInCM(p, y);
        //metric.correction = Point(Construct::XY, 1, dy / dx);
        metric.correction = Point(Construct::XY, dx, dy * dy / dx);
        return metric;
    }

    inline double distanceSquare(const Point& a, const Point& b) const {
        const double dx = (a.x - b.x) * correction.x;
        const double dy = (a.y - b.y) * correction.y;
        return (dx * dx) + (dy * dy);
    }

    inline double distance(const Point& a, const Point& b) const {
        return std::sqrt(distanceSquare(a, b));
    }

    inline double spread(const Rectangle& r, const int dimension) const {
        return r.d(dimension) * correction[dimension];
    }

    Point correction;

};

}
