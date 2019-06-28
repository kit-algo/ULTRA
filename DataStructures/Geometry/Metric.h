/**********************************************************************************

 Copyright (c) 2019 Tobias Zündorf

 MIT License

 Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation
 files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy,
 modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software
 is furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
 LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR
 IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

**********************************************************************************/

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
