#pragma once

#include <iostream>
#include <vector>
#include <string>
#include <cmath>

#include "Borders.h"
#include "Visualization.h"

#include "../DataStructures/Geometry/Point.h"
#include "../DataStructures/Geometry/Metric.h"
#include "../DataStructures/Geometry/Rectangle.h"

#include "../Helpers/String/String.h"

namespace BoundingBoxes {
    using Point = Geometry::Point;
    using Rectangle = Geometry::Rectangle;

    const Rectangle ChauTestNetwork = Rectangle::BoundingBox(Point(Construct::XY, -1, -1), Point(Construct::XY, 5, 5));
    const Rectangle ExampleNetwork = Rectangle::BoundingBox(Point(Construct::XY, 20, 30), Point(Construct::XY, 150, 120));
    const Rectangle NormalizedNetwork = Rectangle::BoundingBox(Point(Construct::XY, 0, 0), Point(Construct::XY, 1, 1));
    const Rectangle Switzerland = Rectangle::BoundingBox(Point(Construct::XY, 5.826, 45.487), Point(Construct::XY, 10.819, 48.142));
    const Rectangle Bern = Rectangle::BoundingBox(Point(Construct::XY, 7.307, 46.868), Point(Construct::XY, 7.563, 47.085));
    const Rectangle Netherlands = Rectangle::BoundingBox(Point(Construct::XY, 2.471, 50.301), Point(Construct::XY, 7.64, 53.8119));
    const Rectangle Germany = Rectangle::BoundingBox(Point(Construct::XY, 5.730, 47.160), Point(Construct::XY, 15.130, 55.070));
    const Rectangle BadenWuerttemberg = Rectangle::BoundingBox(Point(Construct::XY, 7.511, 47.534), Point(Construct::XY, 10.492, 49.791));
    const Rectangle Stuttgart = Rectangle::BoundingBox(Point(Construct::XY, 7.338, 47.072), Point(Construct::XY, 11.788, 50.175));
    const Rectangle England = Rectangle::BoundingBox(Point(Construct::XY, -7.642, 49.940), Point(Construct::XY, 1.947, 58.702));
    const Rectangle London = Rectangle::BoundingBox(Point(Construct::XY, -0.612, 51.233), Point(Construct::XY, 0.715, 51.707));
    const Rectangle Europe = Rectangle::Union<Rectangle>({Switzerland, Germany, Netherlands, England});

    const std::vector<Rectangle> List = {ChauTestNetwork, ExampleNetwork, NormalizedNetwork, Switzerland, Bern, Germany, Netherlands, BadenWuerttemberg, Stuttgart, London};

    inline Rectangle bestMatch(const Rectangle boundingBox) noexcept {
        size_t i = 0;
        for (size_t j = 0; j < List.size(); j++) {
            const double iArea = Rectangle::Intersection(List[i], boundingBox).area();
            const double jArea = Rectangle::Intersection(List[j], boundingBox).area();
            if (jArea > iArea || (jArea == iArea && List[j].area() < List[i].area())) i = j;
        }
        return List[i];
    }

    inline Rectangle bestMatch(const std::vector<Point>& coordinates) noexcept {
        size_t i = 0;
        std::vector<int> counts(List.size(), 0);
        for (size_t j = 0; j < List.size(); j++) {
            for (const Point& coordinate : coordinates) {
                if (List[j].contains(coordinate)) counts[j]++;
            }
            if (counts[i] < counts[j] || (counts[i] == counts[j] && List[j].area() < List[i].area())) i = j;
        }
        return List[i];
    }
}

template<typename DOCUMENT_TYPE>
class MapVisualization : public Visualization<DOCUMENT_TYPE> {

public:
    using DocumentType = DOCUMENT_TYPE;
    using Type = MapVisualization<DocumentType>;
    using Point = Geometry::Point;
    using Rectangle = Geometry::Rectangle;

private:
    using Super = Visualization<DocumentType>;

public:
    MapVisualization(const std::string& fileName, const int width, const int height, const Rectangle& boundingBox) :
        Super(fileName, width, height, boundingBox) {
    }

    MapVisualization(const std::string& fileName, const Rectangle& boundingBox) :
        Super(fileName, DynamicWidth(4000, boundingBox), 4000, boundingBox) {
    }

    MapVisualization(const std::string& fileName, const Rectangle& boundingBox, const double sideBarWidth) :
        Type(fileName, AddSideBar(boundingBox, sideBarWidth)) {
    }

    MapVisualization(const std::string& fileName, const std::vector<Point>& coordinates, const double sideBarWidth = 0.0) :
        Type(fileName, Geometry::Rectangle::BoundingBox(coordinates), sideBarWidth) {
    }

    inline void drawPoints(const std::vector<Point>& points, const Color& color, const Icon icon, const double size, const std::string& caption = "Points") noexcept {
        for (const Point point : points) {
            Super::drawPoint(point, color, icon, size);
        }
        if (caption != "") {
            Super::write(color, icon);
            Super::write(caption + " : " + String::prettyInt(points.size()) + "\n", color);
        }
    }
    inline void drawPoints(const std::vector<Point>& points, const Color& color, const Icon icon, const std::string& caption = "Points") noexcept {drawPoints(points, color, icon, Super::defaultStroke * 5, caption);}
    inline void drawPoints(const std::vector<Point>& points, const Color& color, const double size, const std::string& caption = "Points") noexcept {drawPoints(points, color, Super::defaultIcon, size, caption);}
    inline void drawPoints(const std::vector<Point>& points, const Color& color, const std::string& caption = "Points") noexcept {drawPoints(points, color, Super::defaultIcon, Super::defaultStroke * 5, caption);}
    inline void drawPoints(const std::vector<Point>& points, const Icon icon, const double size, const std::string& caption = "Points") noexcept {drawPoints(points, Super::defaultColor, icon, size, caption);}
    inline void drawPoints(const std::vector<Point>& points, const Icon icon, const std::string& caption = "Points") noexcept {drawPoints(points, Super::defaultColor, icon, Super::defaultStroke * 5, caption);}
    inline void drawPoints(const std::vector<Point>& points, const double size, const std::string& caption = "Points") noexcept {drawPoints(points, Super::defaultColor, Super::defaultIcon, size, caption);}
    inline void drawPoints(const std::vector<Point>& points, const std::string& caption = "Points") noexcept {drawPoints(points, Super::defaultColor, Super::defaultIcon, Super::defaultStroke * 5, caption);}

    inline void drawBorders(const std::string& country) noexcept {
        if (country == "switzerland") {
            Super::drawLine(Borders::Switzerland, Color::Black, 8, false);
        }
        if (country == "germany") {
            for (const Border& border : Borders::GermanStates) {
                Super::drawLine(border, Color::Grey, 5, false);
            }
            Super::drawLine(Borders::Germany, Color::Black, 8, false);
        }
    }

private:
    inline static double DynamicWidth(const int height, const Rectangle& boundingBox) noexcept {
        const Geometry::GeoMetric metric;
        return (height * metric.spread(boundingBox, 1)) / (metric.spread(boundingBox, 0));
    }

    inline static Rectangle AddSideBar(const Rectangle& boundingBox, const double sideBarWidth) noexcept {
        Rectangle result = boundingBox;
        result.min.x -= sideBarWidth * result.dx();
        return result;
    }

};
