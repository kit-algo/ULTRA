#pragma once

#include <iostream>
#include <vector>
#include <string>
#include <cmath>

#include "MapVisualization.h"

#include "../Helpers/Ranges/Range.h"
#include "../Helpers/Vector/Permutation.h"

#include "../DataStructures/RAPTOR/Data.h"
#include "../DataStructures/Intermediate/Data.h"
#include "../DataStructures/GTFS/Entities/Route.h"

#include "../DataStructures/Partition/VertexPartition.h"
#include "../DataStructures/Partition/NestedDissection.h"

#include "../Algorithms/MaxFlowMinCut/Dinic.h"

template<typename DOCUMENT_TYPE>
class TimeTableVisualization : public MapVisualization<DOCUMENT_TYPE> {

public:
    using DocumentType = DOCUMENT_TYPE;
    using Type = TimeTableVisualization<DocumentType>;
    using Point = Geometry::Point;
    using Rectangle = Geometry::Rectangle;

private:
    using Super = MapVisualization<DocumentType>;

private:
    TimeTableVisualization(const std::string& fileName, const RAPTOR::Data& data, const Rectangle& boundingBox) :
        Super(fileName, boundingBox),
        data(data) {
    }
    TimeTableVisualization(const std::string& fileName, const RAPTOR::Data& data, const std::vector<Point>& coordinates, const double sideBarWidth) :
        Super(fileName, coordinates, sideBarWidth),
        data(data) {
    }

public:
    inline static Type FromBoundingBox(const std::string& fileName, const RAPTOR::Data& data, Rectangle boundingBox) noexcept {
        return Type(fileName, data, boundingBox);
    }
    inline static Type FromRAPTOR(const std::string& fileName, const RAPTOR::Data& data, const double sideBarWidth = 0.0) noexcept {
        return Type(fileName, data, data.getCoordinates(), sideBarWidth);
    }
    inline static Type FromIntermediate(const std::string& fileName, const Intermediate::Data& inter, const double sideBarWidth = 0.3) noexcept {
        RAPTOR::Data data = RAPTOR::Data::FromIntermediate(inter);
        return Type(fileName, data, data.getCoordinates(), sideBarWidth);
    }

    inline void drawRoute(const RouteId route, const Color& color, const double stroke) {
        std::vector<Point> points;
        for (const StopId stop : data.stopsOfRoute(route)) {
            points.emplace_back(data.stopData[stop].coordinates);
        }
        Super::drawLine(points, color, stroke);
    }
    inline void drawRoute(const RouteId route, const Color& color) {drawRoute(route, color, Super::defaultStroke);}
    inline void drawRoute(const RouteId route, const double stroke) {drawRoute(route, Super::defaultColor, stroke);}
    inline void drawRoute(const RouteId route) {drawRoute(route, Super::defaultColor, Super::defaultStroke);}

    inline void drawRoutes(const Color& color, const double stroke, const std::string& caption = "Routes") noexcept {
        for (const RouteId route : data.routes()) {
            drawRoute(route, color, stroke);
        }
        if (caption != "") Super::write(caption + ": " + String::prettyInt(data.numberOfRoutes()) + "\n", color);
    }
    inline void drawRoutes(const Color& color, const std::string& caption = "Routes") noexcept {drawRoutes(color, Super::defaultStroke, caption);}
    inline void drawRoutes(const double stroke, const std::string& caption = "Routes") noexcept {drawRoutes(Super::defaultColor, stroke, caption);}
    inline void drawRoutes(const std::string& caption = "Routes") noexcept {drawRoutes(Super::defaultColor, Super::defaultStroke, caption);}

    inline void drawRoutes(const std::vector<RouteId>& routes, const Color& color, const double stroke, const std::string& caption = "Routes") noexcept {
        for (const RouteId route : routes) {
            drawRoute(route, color, stroke, caption);
        }
        if (caption != "") Super::write(caption + ": " + String::prettyInt(routes.size()) + "\n", color);
    }
    inline void drawRoutes(const std::vector<RouteId>& routes, const Color& color, const std::string& caption = "Routes") noexcept {drawRoutes(routes, color, Super::defaultStroke, caption);}
    inline void drawRoutes(const std::vector<RouteId>& routes, const double stroke, const std::string& caption = "Routes") noexcept {drawRoutes(routes, Super::defaultColor, stroke, caption);}
    inline void drawRoutes(const std::vector<RouteId>& routes, const std::string& caption = "Routes") noexcept {drawRoutes(routes, Super::defaultColor, Super::defaultStroke, caption);}

    template<typename DRAW>
    inline void drawRoutes(const DRAW& draw, const Color& color, const double stroke, const std::string& caption = "Routes") noexcept {
        size_t routeCount = 0;
        for (const RouteId route : data.routes()) {
            if (draw(route)) {
                drawRoute(route, color, stroke);
                routeCount++;
            }
        }
        if (caption != "") Super::write(caption + ": " + String::prettyInt(routeCount) + "\n", color);
    }
    template<typename DRAW>
    inline void drawRoutes(const DRAW& draw, const Color& color, const std::string& caption = "Routes") noexcept {drawRoutes(draw, color, Super::defaultStroke, caption);}
    template<typename DRAW>
    inline void drawRoutes(const DRAW& draw, const double stroke, const std::string& caption = "Routes") noexcept {drawRoutes(draw, Super::defaultColor, stroke, caption);}
    template<typename DRAW>
    inline void drawRoutes(const DRAW& draw, const std::string& caption = "Routes") noexcept {drawRoutes(draw, Super::defaultColor, Super::defaultStroke, caption);}

    inline void drawEdges(const Color& color, const double stroke, const std::string& caption = "Transfers") noexcept {
        for (const Vertex fromVertex : data.transferGraph.vertices()) {
            for (const Edge edge : data.transferGraph.edgesFrom(fromVertex)) {
                const Vertex toVertex = data.transferGraph.get(ToVertex, edge);
                if (toVertex < fromVertex) continue;
                Super::drawLine(data.transferGraph.get(Coordinates, toVertex), data.transferGraph.get(Coordinates, fromVertex), color, stroke);
            }
        }
        if (caption != "") Super::write(caption + ": " + String::prettyInt(data.transferGraph.numEdges() / 2) + "\n", color);
    }
    inline void drawEdges(const Color& color, const std::string& caption = "Transfers") noexcept {drawEdges(color, Super::defaultStroke, caption);}
    inline void drawEdges(const double stroke, const std::string& caption = "Transfers") noexcept {drawEdges(Super::defaultColor, stroke, caption);}
    inline void drawEdges(const std::string& caption = "Transfers") noexcept {drawEdges(Super::defaultColor, Super::defaultStroke, caption);}

    inline void drawStops(const Color& color, const Icon icon, const double size, const std::string& caption, const bool printStopId = false) noexcept {
        for (const StopId stop : data.stops()) {
            Super::drawPoint(data.stopData[stop].coordinates, color, icon, size);
            if (printStopId) Super::write(std::to_string(stop), data.stopData[stop].coordinates + Geometry::Point(Construct::XY, Super::ex(size / 2.0), Super::ey(size / 2.0)), color, size * 2);
        }
        if (caption != "") Super::write(caption + ": " + String::prettyInt(data.numberOfStops()) + "\n", color);
    }
    inline void drawStops(const Color& color, const Icon icon, const double size, const bool printStopId = false) noexcept {drawStops(color, icon, size, "Stops", printStopId);}
    inline void drawStops(const Color& color, const Icon icon, const std::string& caption, const bool printStopId = false) noexcept {drawStops(color, icon, Super::defaultStroke * 5, caption, printStopId);}
    inline void drawStops(const Color& color, const Icon icon, const bool printStopId = false) noexcept {drawStops(color, icon, Super::defaultStroke * 5, "Stops", printStopId);}
    inline void drawStops(const Color& color, const double size, const std::string& caption, const bool printStopId = false) noexcept {drawStops(color, Super::defaultIcon, size, caption, printStopId);}
    inline void drawStops(const Color& color, const double size, const bool printStopId = false) noexcept {drawStops(color, Super::defaultIcon, size, "Stops", printStopId);}
    inline void drawStops(const Color& color, const std::string& caption, const bool printStopId = false) noexcept {drawStops(color, Super::defaultIcon, Super::defaultStroke * 5, caption, printStopId);}
    inline void drawStops(const Color& color, const bool printStopId = false) noexcept {drawStops(color, Super::defaultIcon, Super::defaultStroke * 5, "Stops", printStopId);}
    inline void drawStops(const Icon icon, const double size, const std::string& caption, const bool printStopId = false) noexcept {drawStops(Super::defaultColor, icon, size, caption, printStopId);}
    inline void drawStops(const Icon icon, const double size, const bool printStopId = false) noexcept {drawStops(Super::defaultColor, icon, size, "Stops", printStopId);}
    inline void drawStops(const Icon icon, const std::string& caption, const bool printStopId = false) noexcept {drawStops(Super::defaultColor, icon, Super::defaultStroke * 5, caption, printStopId);}
    inline void drawStops(const Icon icon, const bool printStopId = false) noexcept {drawStops(Super::defaultColor, icon, Super::defaultStroke * 5, "Stops", printStopId);}
    inline void drawStops(const double size, const std::string& caption, const bool printStopId = false) noexcept {drawStops(Super::defaultColor, Super::defaultIcon, size, caption, printStopId);}
    inline void drawStops(const double size, const bool printStopId = false) noexcept {drawStops(Super::defaultColor, Super::defaultIcon, size, "Stops", printStopId);}
    inline void drawStops(const std::string& caption, const bool printStopId = false) noexcept {drawStops(Super::defaultColor, Super::defaultIcon, Super::defaultStroke * 5, caption, printStopId);}
    inline void drawStops(const bool printStopId = false) noexcept {drawStops(Super::defaultColor, Super::defaultIcon, Super::defaultStroke * 5, "Stops", printStopId);}

    inline void drawStops(const std::vector<StopId>& stops, const Color& color, const Icon icon, const double size, const std::string& caption = "Stops") noexcept {
        for (const StopId stop : stops) {
            Super::drawPoint(data.getCoordinates()[stop], color, icon, size);
        }
        if (caption != "") Super::write(caption + " : " + String::prettyInt(stops.size()) + "\n", color);
    }
    inline void drawStops(const std::vector<StopId>& stops, const Color& color, const Icon icon, const std::string& caption = "Stops") noexcept {drawStops(stops, color, icon, Super::defaultStroke * 5, caption);}
    inline void drawStops(const std::vector<StopId>& stops, const Color& color, const double size, const std::string& caption = "Stops") noexcept {drawStops(stops, color, Super::defaultIcon, size, caption);}
    inline void drawStops(const std::vector<StopId>& stops, const Color& color, const std::string& caption = "Stops") noexcept {drawStops(stops, color, Super::defaultIcon, Super::defaultStroke * 5, caption);}
    inline void drawStops(const std::vector<StopId>& stops, const Icon icon, const double size, const std::string& caption = "Stops") noexcept {drawStops(stops, Super::defaultColor, icon, size, caption);}
    inline void drawStops(const std::vector<StopId>& stops, const Icon icon, const std::string& caption = "Stops") noexcept {drawStops(stops, Super::defaultColor, icon, Super::defaultStroke * 5, caption);}
    inline void drawStops(const std::vector<StopId>& stops, const double size, const std::string& caption = "Stops") noexcept {drawStops(stops, Super::defaultColor, Super::defaultIcon, size, caption);}
    inline void drawStops(const std::vector<StopId>& stops, const std::string& caption = "Stops") noexcept {drawStops(stops, Super::defaultColor, Super::defaultIcon, Super::defaultStroke * 5, caption);}

    template<typename DRAW>
    inline void drawStops(const DRAW& draw, const Color& color, const Icon icon, const double size, const std::string& caption = "Stops") noexcept {
        size_t stopCount = 0;
        for (const StopId stop : data.stops()) {
            if (draw(stop)) {
                Super::drawPoint(data.getCoordinates()[stop], color, icon, size);
                stopCount++;
            }
        }
        if (caption != "") Super::write(caption + ": " + String::prettyInt(stopCount) + "\n", color);
    }
    template<typename DRAW>
    inline void drawStops(const DRAW& draw, const Color& color, const Icon icon, const std::string& caption = "Stops") noexcept {drawStops(draw, color, icon, Super::defaultStroke * 5, caption);}
    template<typename DRAW>
    inline void drawStops(const DRAW& draw, const Color& color, const double size, const std::string& caption = "Stops") noexcept {drawStops(draw, color, Super::defaultIcon, size, caption);}
    template<typename DRAW>
    inline void drawStops(const DRAW& draw, const Color& color, const std::string& caption = "Stops") noexcept {drawStops(draw, color, Super::defaultIcon, Super::defaultStroke * 5, caption);}
    template<typename DRAW>
    inline void drawStops(const DRAW& draw, const Icon icon, const double size, const std::string& caption = "Stops") noexcept {drawStops(draw, Super::defaultColor, icon, size, caption);}
    template<typename DRAW>
    inline void drawStops(const DRAW& draw, const Icon icon, const std::string& caption = "Stops") noexcept {drawStops(draw, Super::defaultColor, icon, Super::defaultStroke * 5, caption);}
    template<typename DRAW>
    inline void drawStops(const DRAW& draw, const double size, const std::string& caption = "Stops") noexcept {drawStops(draw, Super::defaultColor, Super::defaultIcon, size, caption);}
    template<typename DRAW>
    inline void drawStops(const DRAW& draw, const std::string& caption = "Stops") noexcept {drawStops(draw, Super::defaultColor, Super::defaultIcon, Super::defaultStroke * 5, caption);}

    template<typename RANGE>
    inline void drawVertices(const RANGE& vertices, const Color& color, const Icon icon, const double size, const std::string& caption = "Stops") noexcept {
        for (const Vertex vertex : vertices) {
            Super::drawPoint(data.getCoordinates()[vertex], color, icon, size);
        }
        if (caption != "") Super::write(caption + " : " + String::prettyInt(vertices.size()) + "\n", color);
    }
    template<typename RANGE>
    inline void drawVertices(const RANGE& vertices, const Color& color, const Icon icon, const std::string& caption = "Stops") noexcept {drawVertices(vertices, color, icon, Super::defaultStroke * 5, caption);}
    template<typename RANGE>
    inline void drawVertices(const RANGE& vertices, const Color& color, const double size, const std::string& caption = "Stops") noexcept {drawVertices(vertices, color, Super::defaultIcon, size, caption);}
    template<typename RANGE>
    inline void drawVertices(const RANGE& vertices, const Color& color, const std::string& caption = "Stops") noexcept {drawVertices(vertices, color, Super::defaultIcon, Super::defaultStroke * 5, caption);}
    template<typename RANGE>
    inline void drawVertices(const RANGE& vertices, const Icon icon, const double size, const std::string& caption = "Stops") noexcept {drawVertices(vertices, Super::defaultColor, icon, size, caption);}
    template<typename RANGE>
    inline void drawVertices(const RANGE& vertices, const Icon icon, const std::string& caption = "Stops") noexcept {drawVertices(vertices, Super::defaultColor, icon, Super::defaultStroke * 5, caption);}
    template<typename RANGE>
    inline void drawVertices(const RANGE& vertices, const double size, const std::string& caption = "Stops") noexcept {drawVertices(vertices, Super::defaultColor, Super::defaultIcon, size, caption);}
    template<typename RANGE>
    inline void drawVertices(const RANGE& vertices, const std::string& caption = "Stops") noexcept {drawVertices(vertices, Super::defaultColor, Super::defaultIcon, Super::defaultStroke * 5, caption);}

    inline void drawRoutesByType() noexcept {
        for (const int type : GTFS::Types) {
            drawRoutes([&](const RouteId route){
                return data.routeData[route].type == type;
            }, colors[type], 3, GTFS::TypeNames[type]);
        }
    }

    inline void drawRoutesByType(const int type) noexcept {
        if (type == GTFS::Type::Undefined) {
            drawRoutes([&](const RouteId route){
                return data.routeData[route].type == type;
            }, Color::Red, 3, "Undefined");
        } else {
            drawRoutes([&](const RouteId route){
                return data.routeData[route].type == type;
            }, colors[type], 3, GTFS::TypeNames[type]);
        }
    }

    inline void drawPartition(const VertexPartition& partition, const std::vector<int>& cellColors, const bool stopsOnly = true) noexcept {
        for (const size_t cell : range(partition.numberOfCells())) {
            if (stopsOnly) {
                drawStops([&](const StopId stop){
                    return (size_t(partition.getCellIdOfVertex(stop)) == cell);
                }, cyclicColor(cellColors[cell]), Icon::Dot, 10, "Cell " + std::to_string(cell));
            } else {
                drawVertices(partition.getCell(cell), cyclicColor(cellColors[cell]), Icon::Dot, 10, "Cell " + std::to_string(cell));
            }
        }
        drawStops([&](const StopId stop){
            return (!partition.hasCell(stop));
        }, Color::Grey, Icon::Dot, 10, "Unassigned");
    }

    inline void drawPartition(const VertexPartition& partition, const bool stopsOnly = true) noexcept {
        drawPartition(partition, Vector::id<int>(partition.numberOfCells()), stopsOnly);
    }

    inline void drawPartition(const NestedDissection& partition, const bool drawBackgroundRoutes = false) noexcept {
        if (drawBackgroundRoutes) {
            for (const int type : GTFS::Types) {
                drawRoutes([&](const RouteId route){
                    return data.routeData[route].type == type;
                }, Color::Grey, 3, "");
            }
        }
        for (const size_t cell : range(partition.firstCellOfLevel(partition.numberOfLevels() - 1))) {
            drawVertices(partition.getSeparatorOfCell(cell), cyclicColor(cell), cyclicIcon(partition.levelOfCell(cell)), 30, "Level " + std::to_string(partition.levelOfCell(cell)) + " separator of cell " + std::to_string(cell));
        }
    }

    template<typename CUT_EDGE>
    inline void drawCutEdges(const std::vector<CUT_EDGE>& cutEdges) noexcept {
        for (const CUT_EDGE edge : cutEdges) {
            Super::drawLine(data.getCoordinates()[edge.from], data.getCoordinates()[edge.to], Color::Black, Super::defaultStroke);
        }
        Super::write("Cut Edges : " + String::prettyInt(cutEdges.size()) + "\n", Color::Black);
    }

    inline void drawJourney(const RAPTOR::Journey& journey) noexcept {
        for (const RAPTOR::JourneyLeg& leg : journey) {
            if (leg.usesRoute) {
                drawRoute(leg.routeId, Color::LightGrey);
                Super::drawLine(data.getCoordinates()[leg.from], data.getCoordinates()[leg.to], colors[data.routeData[leg.routeId].type]);
            } else if (leg.from != leg.to) {
                Super::drawLine(data.getCoordinates()[leg.from], data.getCoordinates()[leg.to], Color::Grey);
            }
        }
        for (const RAPTOR::JourneyLeg& leg : journey) {
            if (leg.from == leg.to) {
                Super::drawPoint(data.getCoordinates()[leg.from], Color::Black, Icon::Dot, Super::defaultStroke * 5);
            } else {
                Super::drawPoint(data.getCoordinates()[leg.from], Color::Black, Icon::Circle, Super::defaultStroke * 5);
                Super::drawPoint(data.getCoordinates()[leg.to], Color::Black, Icon::Circle, Super::defaultStroke * 5);
            }
        }
        Super::drawPoint(data.getCoordinates()[journey.front().from], Color::Red, Icon::Circle, Super::defaultStroke * 5);
        Super::drawPoint(data.getCoordinates()[journey.back().to], Color::Red, Icon::Circle, Super::defaultStroke * 5);
        Super::write("Departure time: " + String::secToTime(journey.front().departureTime));
        Super::write("\nArrival time: " + String::secToTime(journey.back().arrivalTime));
        Super::write("\nTravel time: " + String::secToString(journey.back().arrivalTime - journey.front().departureTime));
        Super::write("\nJourney Legs: " + std::to_string(journey.size()));
        Super::write("\nUsed trips: " + std::to_string(journey.size() / 2));
    }

private:
    const RAPTOR::Data& data;

};
