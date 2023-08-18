#pragma once

#include <iostream>
#include <vector>
#include <string>
#include <cmath>

#include <cairo.h>

#include "Color.h"

#include "../DataStructures/Geometry/Point.h"
#include "../DataStructures/Geometry/Rectangle.h"
#include "../Helpers/String/String.h"

enum class Icon {
    Circle, Dot, Cross, Plus, Square, Box, Diamond, MalteseCross
};

inline Icon cyclicIcon(const int i) noexcept {
    return static_cast<Icon>(i % 8);
}

enum class XAlignment {
    Left, Center, Right
};

enum class YAlignment {
    Bottom, Center, Top
};

template<typename DOCUMENT_TYPE>
class Visualization : public DOCUMENT_TYPE {

public:
    using DocumentType = DOCUMENT_TYPE;
    using Type = Visualization<DocumentType>;
    using Point = Geometry::Point;
    using Rectangle = Geometry::Rectangle;

private:
    using Super = DocumentType;

public:
    Visualization(const std::string& filename, const int width, const int height, const Rectangle& boundingBox = Rectangle::BoundingBox(Point(Construct::XY, 0, 0), Point(Construct::XY, 1, 1))) :
        Super(filename, width, height),
        defaultStroke(5),
        defaultSize(64),
        defaultColor(Color::Black),
        defaultIcon(Icon::Circle),
        defaultXAlignment(XAlignment::Left),
        defaultYAlignment(YAlignment::Bottom),
        defaultFont("Sans"),
        defaultTextOffset(Point(Construct::XY, 0, 0)),
        defaultTextPosition(Point(Construct::XY, 50, 114)),
        defaultMargin(Point(Construct::XY, 50, 114)) {
        setBoundingBox(boundingBox);
        cairo_set_line_cap(Super::context, CAIRO_LINE_CAP_ROUND);
        cairo_set_line_join(Super::context, CAIRO_LINE_JOIN_ROUND);
    }

    inline void write(const std::string& text, const Color& color, const double size, const std::string& font) noexcept {
        cairo_text_extents_t textExtents;
        cairo_set_source_rgba(Super::context, color.r, color.g, color.b, color.a);
        cairo_select_font_face(Super::context, font.c_str(), CAIRO_FONT_SLANT_NORMAL, CAIRO_FONT_WEIGHT_NORMAL);
        cairo_set_font_size(Super::context, size);
        std::vector<std::string> tokens = String::split(text, '\n');
        for (size_t i = 0; i < tokens.size(); i++) {
            const std::string& token = tokens[i];
            cairo_move_to(Super::context, defaultTextPosition.x, defaultTextPosition.y);
            cairo_show_text(Super::context, token.c_str());
            cairo_text_extents(Super::context, token.c_str(), &textExtents);
            if (i < tokens.size() - 1 || (!text.empty() && text.back() == '\n')) {
                defaultTextPosition.y += (1.2 * size);
                defaultTextPosition.x = defaultMargin.x;
            } else {
                defaultTextPosition.y += textExtents.y_advance;
                defaultTextPosition.x += textExtents.x_advance;
            }
        }
    }
    inline void write(const std::string& text, const Color& color, const double size) noexcept {write(text, color, size, defaultFont);}
    inline void write(const std::string& text, const Color& color, const std::string& font) noexcept {write(text, color, defaultSize, font);}
    inline void write(const std::string& text, const Color& color) noexcept {write(text, color, defaultSize, defaultFont);}
    inline void write(const std::string& text, const double size, const std::string& font) noexcept {write(text, defaultColor, size, font);}
    inline void write(const std::string& text, const double size) noexcept {write(text, defaultColor, size, defaultFont);}
    inline void write(const std::string& text, const std::string& font) noexcept {write(text, defaultColor, defaultSize, font);}
    inline void write(const std::string& text) noexcept {write(text, defaultColor, defaultSize, defaultFont);}

    inline void write(const Color& color, const Icon icon, const double size) noexcept {
        drawPoint(abs(defaultTextPosition.x, defaultTextPosition.y - (0.35 * size)), color, icon, size * 0.5);
        defaultTextPosition.x += (0.7 * size);
    }
    inline void write(const Color& color, const Icon icon) noexcept {write(color, icon, defaultSize);}
    inline void write(const Color& color, const double size) noexcept {write(color, defaultIcon, size);}
    inline void write(const Color& color) noexcept {write(color, defaultIcon, defaultSize);}
    inline void write(const Icon icon, const double size) noexcept {write(defaultColor, icon, size);}
    inline void write(const Icon icon) noexcept {write(defaultColor, icon, defaultSize);}
    inline void write(const double size) noexcept {write(defaultColor, defaultIcon, size);}
    inline void write() noexcept {write(defaultColor, defaultIcon, defaultSize);}

    inline void write(const std::string& text, const Point& point, const Color& color, const XAlignment xAlignment, const YAlignment yAlignment, const double size, const std::string& font) noexcept {
        cairo_text_extents_t textExtents;
        cairo_set_source_rgba(Super::context, color.r, color.g, color.b, color.a);
        cairo_select_font_face(Super::context, font.c_str(), CAIRO_FONT_SLANT_NORMAL, CAIRO_FONT_WEIGHT_NORMAL);
        cairo_set_font_size(Super::context, size);
        cairo_text_extents(Super::context, text.c_str(), &textExtents);
        double px = x(point) - textExtents.x_bearing + defaultTextOffset.x;
        switch(xAlignment) {
            case XAlignment::Left: {
                break;
            }
            case XAlignment::Center: {
                px -= textExtents.width / 2.0;
                break;
            }
            case XAlignment::Right: {
                px -= textExtents.width;
                break;
            }
        }
        double py = y(point) + defaultTextOffset.y;
        switch(yAlignment) {
            case YAlignment::Bottom: {
                break;
            }
            case YAlignment::Center: {
                py += textExtents.height / 2.0;
                break;
            }
            case YAlignment::Top: {
                py += textExtents.height;
                break;
            }
        }
        cairo_move_to(Super::context, px, py);
        cairo_show_text(Super::context, text.c_str());
    }
    inline void write(const std::string& text, const Point& point, const Color& color, const XAlignment xAlignment, const YAlignment yAlignment, const double size) noexcept {write(text, point, color, xAlignment, yAlignment, size, defaultFont);}
    inline void write(const std::string& text, const Point& point, const Color& color, const XAlignment xAlignment, const YAlignment yAlignment, const std::string& font) noexcept {write(text, point, color, xAlignment, yAlignment, defaultSize, font);}
    inline void write(const std::string& text, const Point& point, const Color& color, const XAlignment xAlignment, const YAlignment yAlignment)  noexcept {write(text, point, color, xAlignment, yAlignment, defaultSize, defaultFont);}
    inline void write(const std::string& text, const Point& point, const Color& color, const double size, const std::string& font) noexcept {write(text, point, color, defaultXAlignment, defaultYAlignment, size, font);}
    inline void write(const std::string& text, const Point& point, const Color& color, const XAlignment xAlignment, const double size, const std::string& font) noexcept {write(text, point, color, xAlignment, defaultYAlignment, size, font);}
    inline void write(const std::string& text, const Point& point, const Color& color, const YAlignment yAlignment, const double size, const std::string& font) noexcept {write(text, point, color, defaultXAlignment, yAlignment, size, font);}
    inline void write(const std::string& text, const Point& point, const Color& color, const double size) noexcept {write(text, point, color, defaultXAlignment, defaultYAlignment, size, defaultFont);}
    inline void write(const std::string& text, const Point& point, const Color& color, const XAlignment xAlignment, const double size) noexcept {write(text, point, color, xAlignment, defaultYAlignment, size, defaultFont);}
    inline void write(const std::string& text, const Point& point, const Color& color, const YAlignment yAlignment, const double size) noexcept {write(text, point, color, defaultXAlignment, yAlignment, size, defaultFont);}
    inline void write(const std::string& text, const Point& point, const Color& color, const std::string& font) noexcept {write(text, point, color, defaultXAlignment, defaultYAlignment, defaultSize, font);}
    inline void write(const std::string& text, const Point& point, const Color& color, const XAlignment xAlignment, const std::string& font) noexcept {write(text, point, color, xAlignment, defaultYAlignment, defaultSize, font);}
    inline void write(const std::string& text, const Point& point, const Color& color, const YAlignment yAlignment, const std::string& font) noexcept {write(text, point, color, defaultXAlignment, yAlignment, defaultSize, font);}
    inline void write(const std::string& text, const Point& point, const Color& color) noexcept {write(text, point, color, defaultXAlignment, defaultYAlignment, defaultSize, defaultFont);}
    inline void write(const std::string& text, const Point& point, const Color& color, const XAlignment xAlignment) noexcept {write(text, point, color, xAlignment, defaultYAlignment, defaultSize, defaultFont);}
    inline void write(const std::string& text, const Point& point, const Color& color, const YAlignment yAlignment) noexcept {write(text, point, color, defaultXAlignment, yAlignment, defaultSize, defaultFont);}
    inline void write(const std::string& text, const Point& point, const XAlignment xAlignment, const YAlignment yAlignment, const double size, const std::string& font) noexcept {write(text, point, defaultColor, xAlignment, yAlignment, size, font);}
    inline void write(const std::string& text, const Point& point, const XAlignment xAlignment, const YAlignment yAlignment, const double size) noexcept {write(text, point, defaultColor, xAlignment, yAlignment, size, defaultFont);}
    inline void write(const std::string& text, const Point& point, const XAlignment xAlignment, const YAlignment yAlignment, const std::string& font) noexcept {write(text, point, defaultColor, xAlignment, yAlignment, defaultSize, font);}
    inline void write(const std::string& text, const Point& point, const XAlignment xAlignment, const YAlignment yAlignment) noexcept {write(text, point, defaultColor, xAlignment, yAlignment, defaultSize, defaultFont);}
    inline void write(const std::string& text, const Point& point, const double size, const std::string& font) noexcept {write(text, point, defaultColor, defaultXAlignment, defaultYAlignment, size, font);}
    inline void write(const std::string& text, const Point& point, const XAlignment xAlignment, const double size, const std::string& font) noexcept {write(text, point, defaultColor, xAlignment, defaultYAlignment, size, font);}
    inline void write(const std::string& text, const Point& point, const YAlignment yAlignment, const double size, const std::string& font) noexcept {write(text, point, defaultColor, defaultXAlignment, yAlignment, size, font);}
    inline void write(const std::string& text, const Point& point, const double size) noexcept {write(text, point, defaultColor, defaultXAlignment, defaultYAlignment, size, defaultFont);}
    inline void write(const std::string& text, const Point& point, const XAlignment xAlignment, const double size) noexcept {write(text, point, defaultColor, xAlignment, defaultYAlignment, size, defaultFont);}
    inline void write(const std::string& text, const Point& point, const YAlignment yAlignment, const double size) noexcept {write(text, point, defaultColor, defaultXAlignment, yAlignment, size, defaultFont);}
    inline void write(const std::string& text, const Point& point, const std::string& font) noexcept {write(text, point, defaultColor, defaultXAlignment, defaultYAlignment, defaultSize, font);}
    inline void write(const std::string& text, const Point& point, const XAlignment xAlignment, const std::string& font) noexcept {write(text, point, defaultColor, xAlignment, defaultYAlignment, defaultSize, font);}
    inline void write(const std::string& text, const Point& point, const YAlignment yAlignment, const std::string& font) noexcept {write(text, point, defaultColor, defaultXAlignment, yAlignment, defaultSize, font);}
    inline void write(const std::string& text, const Point& point) noexcept {write(text, point, defaultColor, defaultXAlignment, defaultYAlignment, defaultSize, defaultFont);}
    inline void write(const std::string& text, const Point& point, const XAlignment xAlignment) noexcept {write(text, point, defaultColor, xAlignment, defaultYAlignment, defaultSize, defaultFont);}
    inline void write(const std::string& text, const Point& point, const YAlignment yAlignment) noexcept {write(text, point, defaultColor, defaultXAlignment, yAlignment, defaultSize, defaultFont);}

    inline void write(const std::string& text, const double px, const double py, const Color& color, const XAlignment xAlignment, const YAlignment yAlignment, const double size, const std::string& font) noexcept {write(text, Point(Construct::XY, px, py), color, xAlignment, yAlignment, size, font);}
    inline void write(const std::string& text, const double px, const double py, const Color& color, const XAlignment xAlignment, const YAlignment yAlignment, const double size) noexcept {write(text, Point(Construct::XY, px, py), color, xAlignment, yAlignment, size, defaultFont);}
    inline void write(const std::string& text, const double px, const double py, const Color& color, const XAlignment xAlignment, const YAlignment yAlignment, const std::string& font) noexcept {write(text, Point(Construct::XY, px, py), color, xAlignment, yAlignment, defaultSize, font);}
    inline void write(const std::string& text, const double px, const double py, const Color& color, const XAlignment xAlignment, const YAlignment yAlignment) noexcept {write(text, Point(Construct::XY, px, py), color, xAlignment, yAlignment, defaultSize, defaultFont);}
    inline void write(const std::string& text, const double px, const double py, const Color& color, const double size, const std::string& font) noexcept {write(text, Point(Construct::XY, px, py), color, defaultXAlignment, defaultYAlignment, size, font);}
    inline void write(const std::string& text, const double px, const double py, const XAlignment xAlignment, const Color& color, const double size, const std::string& font) noexcept {write(text, Point(Construct::XY, px, py), color, xAlignment, defaultYAlignment, size, font);}
    inline void write(const std::string& text, const double px, const double py, const YAlignment yAlignment, const Color& color, const double size, const std::string& font) noexcept {write(text, Point(Construct::XY, px, py), color, defaultXAlignment, yAlignment, size, font);}
    inline void write(const std::string& text, const double px, const double py, const Color& color, const double size) noexcept {write(text, Point(Construct::XY, px, py), color, defaultXAlignment, defaultYAlignment, size, defaultFont);}
    inline void write(const std::string& text, const double px, const double py, const XAlignment xAlignment, const Color& color, const double size) noexcept {write(text, Point(Construct::XY, px, py), color, xAlignment, defaultYAlignment, size, defaultFont);}
    inline void write(const std::string& text, const double px, const double py, const YAlignment yAlignment, const Color& color, const double size) noexcept {write(text, Point(Construct::XY, px, py), color, defaultXAlignment, yAlignment, size, defaultFont);}
    inline void write(const std::string& text, const double px, const double py, const Color& color,  const std::string& font) noexcept {write(text, Point(Construct::XY, px, py), color, defaultXAlignment, defaultYAlignment, defaultSize, font);}
    inline void write(const std::string& text, const double px, const double py, const XAlignment xAlignment, const Color& color,  const std::string& font) noexcept {write(text, Point(Construct::XY, px, py), color, xAlignment, defaultYAlignment, defaultSize, font);}
    inline void write(const std::string& text, const double px, const double py, const YAlignment yAlignment, const Color& color,  const std::string& font) noexcept {write(text, Point(Construct::XY, px, py), color, defaultXAlignment, yAlignment, defaultSize, font);}
    inline void write(const std::string& text, const double px, const double py, const Color& color) noexcept {write(text, Point(Construct::XY, px, py), color, defaultXAlignment, defaultYAlignment, defaultSize, defaultFont);}
    inline void write(const std::string& text, const double px, const double py, const XAlignment xAlignment, const Color& color) noexcept {write(text, Point(Construct::XY, px, py), color, xAlignment, defaultYAlignment, defaultSize, defaultFont);}
    inline void write(const std::string& text, const double px, const double py, const YAlignment yAlignment, const Color& color) noexcept {write(text, Point(Construct::XY, px, py), color, defaultXAlignment, yAlignment, defaultSize, defaultFont);}
    inline void write(const std::string& text, const double px, const double py, const XAlignment xAlignment, const YAlignment yAlignment, const double size, const std::string& font) noexcept {write(text, Point(Construct::XY, px, py), defaultColor, xAlignment, yAlignment, size, font);}
    inline void write(const std::string& text, const double px, const double py, const XAlignment xAlignment, const YAlignment yAlignment, const double size) noexcept {write(text, Point(Construct::XY, px, py), defaultColor, xAlignment, yAlignment, size, defaultFont);}
    inline void write(const std::string& text, const double px, const double py, const XAlignment xAlignment, const YAlignment yAlignment, const std::string& font) noexcept {write(text, Point(Construct::XY, px, py), defaultColor, xAlignment, yAlignment, defaultSize, font);}
    inline void write(const std::string& text, const double px, const double py, const XAlignment xAlignment, const YAlignment yAlignment) noexcept {write(text, Point(Construct::XY, px, py), defaultColor, xAlignment, yAlignment, defaultSize, defaultFont);}
    inline void write(const std::string& text, const double px, const double py, const double size, const std::string& font) noexcept {write(text, Point(Construct::XY, px, py), defaultColor, defaultXAlignment, defaultYAlignment, size, font);}
    inline void write(const std::string& text, const double px, const double py, const XAlignment xAlignment, const double size, const std::string& font) noexcept {write(text, Point(Construct::XY, px, py), defaultColor, xAlignment, defaultYAlignment, size, font);}
    inline void write(const std::string& text, const double px, const double py, const YAlignment yAlignment, const double size, const std::string& font) noexcept {write(text, Point(Construct::XY, px, py), defaultColor, defaultXAlignment, yAlignment, size, font);}
    inline void write(const std::string& text, const double px, const double py, const double size) noexcept {write(text, Point(Construct::XY, px, py), defaultColor, defaultXAlignment, defaultYAlignment, size, defaultFont);}
    inline void write(const std::string& text, const double px, const double py, const XAlignment xAlignment, const double size) noexcept {write(text, Point(Construct::XY, px, py), defaultColor, xAlignment, defaultYAlignment, size, defaultFont);}
    inline void write(const std::string& text, const double px, const double py, const YAlignment yAlignment, const double size) noexcept {write(text, Point(Construct::XY, px, py), defaultColor, defaultXAlignment, yAlignment, size, defaultFont);}
    inline void write(const std::string& text, const double px, const double py, const std::string& font) noexcept {write(text, Point(Construct::XY, px, py), defaultColor, defaultXAlignment, defaultYAlignment, defaultSize, font);}
    inline void write(const std::string& text, const double px, const double py, const XAlignment xAlignment, const std::string& font) noexcept {write(text, Point(Construct::XY, px, py), defaultColor, xAlignment, defaultYAlignment, defaultSize, font);}
    inline void write(const std::string& text, const double px, const double py, const YAlignment yAlignment, const std::string& font) noexcept {write(text, Point(Construct::XY, px, py), defaultColor, defaultXAlignment, yAlignment, defaultSize, font);}
    inline void write(const std::string& text, const double px, const double py) noexcept {write(text, Point(Construct::XY, px, py), defaultColor, defaultXAlignment, defaultYAlignment, defaultSize, defaultFont);}
    inline void write(const std::string& text, const double px, const double py, const XAlignment xAlignment) noexcept {write(text, Point(Construct::XY, px, py), defaultColor, xAlignment, defaultYAlignment, defaultSize, defaultFont);}
    inline void write(const std::string& text, const double px, const double py, const YAlignment yAlignment) noexcept {write(text, Point(Construct::XY, px, py), defaultColor, defaultXAlignment, yAlignment, defaultSize, defaultFont);}

    inline void drawPoint(const Point& point, const Color& color, const Icon icon, const double size) noexcept {
        if (!contains(point)) return;
        cairo_set_source_rgba(Super::context, color.r, color.g, color.b, color.a);
        switch(icon) {
            case Icon::Circle: {
                const double radius = size * 2.0 / 5.0;
                cairo_set_line_width(Super::context, size / 5.0);
                cairo_move_to(Super::context, x(point) + radius, y(point));
                cairo_arc(Super::context, x(point), y(point), radius, 0.0, 6.2831);
                cairo_stroke(Super::context);
                break;
            }
            case Icon::Dot: {
                const double radius = size / 2.0;
                cairo_move_to(Super::context, x(point) + radius, y(point));
                cairo_arc(Super::context, x(point), y(point), radius, 0.0, 6.2831);
                cairo_fill(Super::context);
                break;
            }
            case Icon::Cross: {
                const double offset = sqrt(size * size / 8.0);
                cairo_set_line_width(Super::context, size / 5.0);
                cairo_move_to(Super::context, x(point) - offset, y(point) - offset);
                cairo_line_to(Super::context, x(point) + offset, y(point) + offset);
                cairo_stroke(Super::context);
                cairo_move_to(Super::context, x(point) - offset, y(point) + offset);
                cairo_line_to(Super::context, x(point) + offset, y(point) - offset);
                cairo_stroke(Super::context);
                break;
            }
            case Icon::Plus: {
                const double offset = size / 2.0;
                cairo_set_line_width(Super::context, size / 5.0);
                cairo_move_to(Super::context, x(point) - offset, y(point));
                cairo_line_to(Super::context, x(point) + offset, y(point));
                cairo_stroke(Super::context);
                cairo_move_to(Super::context, x(point), y(point) + offset);
                cairo_line_to(Super::context, x(point), y(point) - offset);
                cairo_stroke(Super::context);
                break;
            }
            case Icon::Square: {
                const double offset = 1.1 * sqrt(size * size / 8.0);
                cairo_set_line_width(Super::context, size / 5.0);
                cairo_move_to(Super::context, x(point) - offset, y(point) - offset);
                cairo_line_to(Super::context, x(point) - offset, y(point) + offset);
                cairo_line_to(Super::context, x(point) + offset, y(point) + offset);
                cairo_line_to(Super::context, x(point) + offset, y(point) - offset);
                cairo_line_to(Super::context, x(point) - offset, y(point) - offset);
                cairo_stroke(Super::context);
                break;
            }
            case Icon::Box: {
                const double offset = 0.9 * sqrt(size * size * 8.0 / 25);
                cairo_move_to(Super::context, x(point) - offset, y(point) - offset);
                cairo_line_to(Super::context, x(point) - offset, y(point) + offset);
                cairo_line_to(Super::context, x(point) + offset, y(point) + offset);
                cairo_line_to(Super::context, x(point) + offset, y(point) - offset);
                cairo_line_to(Super::context, x(point) - offset, y(point) - offset);
                cairo_fill(Super::context);
                break;
            }
            case Icon::Diamond: {
                const double offset = size / 2.0;
                cairo_set_line_width(Super::context, size / 5.0);
                cairo_move_to(Super::context, x(point) - offset, y(point));
                cairo_line_to(Super::context, x(point), y(point) + offset);
                cairo_line_to(Super::context, x(point) + offset, y(point));
                cairo_line_to(Super::context, x(point), y(point) - offset);
                cairo_line_to(Super::context, x(point) - offset, y(point));
                cairo_stroke(Super::context);
                break;
            }
            case Icon::MalteseCross: {
                const double offset = size / 2.0;
                cairo_move_to(Super::context, x(point), y(point));
                cairo_line_to(Super::context, x(point) - offset, y(point) - (0.43 * offset));
                cairo_line_to(Super::context, x(point) - (0.73 * offset), y(point));
                cairo_line_to(Super::context, x(point) - offset, y(point) + (0.43 * offset));
                cairo_line_to(Super::context, x(point), y(point));
                cairo_line_to(Super::context, x(point) - (0.43 * offset), y(point) - offset);
                cairo_line_to(Super::context, x(point), y(point) - (0.73 * offset));
                cairo_line_to(Super::context, x(point) + (0.43 * offset), y(point) - offset);
                cairo_line_to(Super::context, x(point), y(point));
                cairo_line_to(Super::context, x(point) + offset, y(point) + (0.43 * offset));
                cairo_line_to(Super::context, x(point) + (0.73 * offset), y(point));
                cairo_line_to(Super::context, x(point) + offset, y(point) - (0.43 * offset));
                cairo_line_to(Super::context, x(point), y(point));
                cairo_line_to(Super::context, x(point) + (0.43 * offset), y(point) + offset);
                cairo_line_to(Super::context, x(point), y(point) + (0.73 * offset));
                cairo_line_to(Super::context, x(point) - (0.43 * offset), y(point) + offset);
                cairo_line_to(Super::context, x(point), y(point));
                cairo_fill(Super::context);
                break;
            }
        }
    }
    inline void drawPoint(const Point& point, const Color& color, const Icon icon) noexcept {drawPoint(point, color, icon, defaultSize);}
    inline void drawPoint(const Point& point, const Color& color, const double size) noexcept {drawPoint(point, color, defaultIcon, size);}
    inline void drawPoint(const Point& point, const Color& color) noexcept {drawPoint(point, color, defaultIcon, defaultSize);}
    inline void drawPoint(const Point& point, const Icon icon, const double size) noexcept {drawPoint(point, defaultColor, icon, size);}
    inline void drawPoint(const Point& point, const Icon icon) noexcept {drawPoint(point, defaultColor, icon, defaultSize);}
    inline void drawPoint(const Point& point, const double size) noexcept {drawPoint(point, defaultColor, defaultIcon, size);}
    inline void drawPoint(const Point& point) noexcept {drawPoint(point, defaultColor, defaultIcon, defaultSize);}

    inline void drawPoint(const double px, const double py, const Color& color, const Icon icon, const double size) noexcept {drawPoint(Point(Construct::XY, px, py), color, icon, size);}
    inline void drawPoint(const double px, const double py, const Color& color, const Icon icon) noexcept {drawPoint(Point(Construct::XY, px, py), color, icon, defaultSize);}
    inline void drawPoint(const double px, const double py, const Color& color, const double size) noexcept {drawPoint(Point(Construct::XY, px, py), color, defaultIcon, size);}
    inline void drawPoint(const double px, const double py, const Color& color) noexcept {drawPoint(Point(Construct::XY, px, py), color, defaultIcon, defaultSize);}
    inline void drawPoint(const double px, const double py, const Icon icon, const double size) noexcept {drawPoint(Point(Construct::XY, px, py), defaultColor, icon, size);}
    inline void drawPoint(const double px, const double py, const Icon icon) noexcept {drawPoint(Point(Construct::XY, px, py), defaultColor, icon, defaultSize);}
    inline void drawPoint(const double px, const double py, const double size) noexcept {drawPoint(Point(Construct::XY, px, py), defaultColor, defaultIcon, size);}
    inline void drawPoint(const double px, const double py) noexcept {drawPoint(Point(Construct::XY, px, py), defaultColor, defaultIcon, defaultSize);}

    inline void drawLine(const Point& from, const Point& to, const Color& color, const double stroke, const bool removeIfClipped = true) noexcept {
        if (removeIfClipped) {
            if (!contains(from)) return;
            if (!contains(to)) return;
        } else {
            if ((!contains(from)) && (!contains(to))) return;
        }
        cairo_set_line_width(Super::context, stroke);
        cairo_set_source_rgba(Super::context, color.r, color.g, color.b, color.a);
        cairo_move_to(Super::context, x(from), y(from));
        cairo_line_to(Super::context, x(to), y(to));
        cairo_stroke(Super::context);
    }
    inline void drawLine(const Point& from, const Point& to, const Color& color, const int stroke, const bool removeIfClipped = true) noexcept {drawLine(from, to, color, static_cast<double>(stroke), removeIfClipped);}
    inline void drawLine(const Point& from, const Point& to, const Color& color, const bool removeIfClipped = true) noexcept {drawLine(from, to, color, defaultStroke, removeIfClipped);}
    inline void drawLine(const Point& from, const Point& to, const double stroke, const bool removeIfClipped = true) noexcept {drawLine(from, to, defaultColor, stroke, removeIfClipped);}
    inline void drawLine(const Point& from, const Point& to, const int stroke, const bool removeIfClipped = true) noexcept {drawLine(from, to, defaultColor, stroke, removeIfClipped);}
    inline void drawLine(const Point& from, const Point& to, const bool removeIfClipped = true) noexcept {drawLine(from, to, defaultColor, defaultStroke, removeIfClipped);}

    inline void drawLine(const double x1, const double y1, const double x2, const double y2, const Color& color, const double stroke) noexcept {drawLine(Point(Construct::XY, x1, y1), Point(Construct::XY, x2, y2), color, stroke);}
    inline void drawLine(const double x1, const double y1, const double x2, const double y2, const Color& color) noexcept {drawLine(Point(Construct::XY, x1, y1), Point(Construct::XY, x2, y2), color, defaultStroke);}
    inline void drawLine(const double x1, const double y1, const double x2, const double y2, const double stroke) noexcept {drawLine(Point(Construct::XY, x1, y1), Point(Construct::XY, x2, y2), defaultColor, stroke);}
    inline void drawLine(const double x1, const double y1, const double x2, const double y2) noexcept {drawLine(Point(Construct::XY, x1, y1), Point(Construct::XY, x2, y2), defaultColor, defaultStroke);}

    inline void drawLine(const std::vector<Point>& points, const Color& color, const double stroke, const bool removeIfClipped = true) noexcept {
        if (removeIfClipped && !contains(points)) return;
        cairo_set_line_width(Super::context, stroke);
        cairo_set_source_rgba(Super::context, color.r, color.g, color.b, color.a);
        cairo_move_to(Super::context, x(points[0].x), y(points[0].y));
        for (size_t i = 1; i < points.size(); i++) {
            cairo_line_to(Super::context, x(points[i].x), y(points[i].y));
        }
        cairo_stroke(Super::context);
    }
    inline void drawLine(const std::vector<Point>& points, const Color& color, const bool removeIfClipped = true) noexcept {drawLine(points, color, defaultStroke, removeIfClipped);}
    inline void drawLine(const std::vector<Point>& points, const double stroke, const bool removeIfClipped = true) noexcept {drawLine(points, defaultColor, stroke, removeIfClipped);}
    inline void drawLine(const std::vector<Point>& points, const bool removeIfClipped = true) noexcept {drawLine(points, defaultColor, defaultStroke, removeIfClipped);}

    inline void fillPolygon(const std::vector<Point>& points, const Color& color, const bool removeIfClipped = true) noexcept {
        if (removeIfClipped && !contains(points)) return;
        cairo_set_source_rgba(Super::context, color.r, color.g, color.b, color.a);
        cairo_move_to(Super::context, x(points[0].x), y(points[0].y));
        for (size_t i = 1; i < points.size(); i++) {
            cairo_line_to(Super::context, x(points[i].x), y(points[i].y));
        }
        cairo_fill(Super::context);
    }
    inline void fillPolygon(const std::vector<Point>& points, const bool removeIfClipped = true) noexcept {fillPolygon(points, defaultColor, removeIfClipped);}

    inline void fillRectangle(const Rectangle& rectangle, const Color& color, const bool removeIfClipped = true) noexcept {
        if (removeIfClipped && !contains(rectangle)) return;
        cairo_set_source_rgba(Super::context, color.r, color.g, color.b, color.a);
        cairo_move_to(Super::context, x(rectangle.min), y(rectangle.min));
        cairo_line_to(Super::context, x(rectangle.min), y(rectangle.max));
        cairo_line_to(Super::context, x(rectangle.max), y(rectangle.max));
        cairo_line_to(Super::context, x(rectangle.max), y(rectangle.min));
        cairo_fill(Super::context);
    }
    inline void fillRectangle(const Rectangle& rectangle, const bool removeIfClipped = true) noexcept {fillRectangle(rectangle, defaultColor, removeIfClipped);}

    inline void drawRectangle(const Rectangle& rectangle, const Color& color, const bool removeIfClipped = true) noexcept {
        if (removeIfClipped && !contains(rectangle)) return;
        cairo_set_source_rgba(Super::context, color.r, color.g, color.b, color.a);
        cairo_move_to(Super::context, x(rectangle.min), y(rectangle.min));
        cairo_line_to(Super::context, x(rectangle.min), y(rectangle.max));
        cairo_line_to(Super::context, x(rectangle.max), y(rectangle.max));
        cairo_line_to(Super::context, x(rectangle.max), y(rectangle.min));
        cairo_line_to(Super::context, x(rectangle.min), y(rectangle.min));
        cairo_stroke(Super::context);
    }
    inline void drawRectangle(const Rectangle& rectangle, const bool removeIfClipped = true) noexcept {drawRectangle(rectangle, defaultColor, removeIfClipped);}

    inline void drawPixel(const double x, const double y, const Color& color) noexcept {
        cairo_set_source_rgba(Super::context, color.r, color.g, color.b, color.a);
        cairo_move_to(Super::context, x, y);
        cairo_line_to(Super::context, x + 1, y);
        cairo_line_to(Super::context, x + 1, y + 1);
        cairo_line_to(Super::context, x, y + 1);
        cairo_fill(Super::context);
    }
    inline void drawPixel(const Point& point, const Color& color) noexcept {drawPixel(point.x, point.y, color);}
    inline void drawPixel(const double x, const double y) noexcept {drawPixel(x, y, defaultColor);}
    inline void drawPixel(const Point& point) noexcept {drawPixel(point.x, point.y, defaultColor);}

    inline void drawPoints(const std::vector<Point>& points, const Color& color, const Icon icon, const double size) noexcept {
        if (!contains(points)) return;
        for (const Point& point : points) {
            drawPoint(point, color, icon, size);
        }
    }
    inline void drawPoints(const std::vector<Point>& points, const Color& color, const Icon icon) noexcept {drawPoints(points, color, icon, defaultSize);}
    inline void drawPoints(const std::vector<Point>& points, const Color& color, const double size) noexcept {drawPoints(points, color, defaultIcon, size);}
    inline void drawPoints(const std::vector<Point>& points, const Color& color) noexcept {drawPoints(points, color, defaultIcon, defaultSize);}
    inline void drawPoints(const std::vector<Point>& points, const Icon icon, const double size) noexcept {drawPoints(points, defaultColor, icon, size);}
    inline void drawPoints(const std::vector<Point>& points, const Icon icon) noexcept {drawPoints(points, defaultColor, icon, defaultSize);}
    inline void drawPoints(const std::vector<Point>& points, const double size) noexcept {drawPoints(points, defaultColor, defaultIcon, size);}
    inline void drawPoints(const std::vector<Point>& points) noexcept {drawPoints(points, defaultColor, defaultIcon, defaultSize);}

    inline void setDashPattern(const std::vector<double>& pattern) noexcept {
        cairo_set_dash(Super::context, pattern.data(), pattern.size(), 0);
    }
    inline void setDottedDashPattern() noexcept {setDashPattern({0, 2 * defaultStroke});}
    inline void SetDashedDashPattern() noexcept {setDashPattern({4 * defaultStroke});}
    inline void clearDashPattern() noexcept {setDashPattern({});}

    inline double textWidth(const std::string text, const double size) noexcept {
        cairo_text_extents_t textExtents;
        cairo_set_font_size(Super::context, size);
        cairo_text_extents(Super::context, text.c_str(), &textExtents);
        return textExtents.width;
    }
    inline double textWidth(const std::string text) noexcept {return textWidth(text, defaultSize);}

    inline double textHeight(const std::string text, const double size) noexcept {
        cairo_text_extents_t textExtents;
        cairo_set_font_size(Super::context, size);
        cairo_text_extents(Super::context, text.c_str(), &textExtents);
        return textExtents.height;
    }
    inline double textHeight(const std::string text) noexcept {return textHeight(text, defaultSize);}

    inline void newPage() noexcept {
        defaultTextPosition = defaultMargin;
        DocumentType::newPage();
    }

    inline void setBoundingBox(const Rectangle& bb) noexcept {
        boundingBox = bb;
        dx = -bb.min.x;
        dy = -bb.max.y;
        fx = Super::width / bb.dx();
        fy = -Super::height / bb.dy();
    }

    inline bool contains(const std::vector<Point>& points) const noexcept {
        for (const Point& point : points) {
            if (!boundingBox.contains(point)) return false;
        }
        return true;
    }
    inline bool contains(const Point& point) const noexcept {return boundingBox.contains(point);}
    inline bool contains(const double px, const double py) const noexcept {return boundingBox.contains(Point(Construct::XY, px, py));}
    inline bool contains(const Rectangle& rectangle) const noexcept {return boundingBox.contains(rectangle);}

    inline int getWidth() const noexcept {
        return Super::width;
    }
    inline int getHeight() const noexcept {
        return Super::height;
    }
    inline XAlignment getDefaultXAlignment() const noexcept {
        return defaultXAlignment;
    }
    inline YAlignment getDefaultYAlignment() const noexcept {
        return defaultYAlignment;
    }
    inline Icon getDefaultIcon() const noexcept {
        return defaultIcon;
    }
    inline const Color& getDefaultColor() const noexcept {
        return defaultColor;
    }
    inline const std::string& getDefaultFont() const noexcept {
        return defaultFont;
    }
    inline double getDefaultSize() const noexcept {
        return defaultSize;
    }
    inline double getDefaultStroke() const noexcept {
        return defaultStroke;
    }
    inline const Point& getDefaultTextOffset() const noexcept {
        return defaultTextOffset;
    }
    inline const Point& getDefaultTextPosition() const noexcept {
        return defaultTextPosition;
    }
    inline const Point& getDefaultMargin() const noexcept {
        return defaultMargin;
    }
    inline const Rectangle& getBoundingBox() const noexcept {
        return boundingBox;
    }

    inline void setDefaultXAlignment(const XAlignment alignment) noexcept {
        defaultXAlignment = alignment;
    }
    inline void setDefaultYAlignment(const YAlignment alignment) noexcept {
        defaultYAlignment = alignment;
    }
    inline void setDefaultIcon(const Icon icon) noexcept {
        defaultIcon = icon;
    }
    inline void setDefaultColor(const Color& color) noexcept {
        defaultColor = color;
    }
    inline void setDefaultFont(const std::string& font) noexcept {
        defaultFont = font;
    }
    inline void setDefaultSize(const double size) noexcept {
        defaultSize = size;
    }
    inline void setDefaultStroke(double stroke) noexcept {
        defaultStroke = stroke;
    }
    inline void setDefaultTextOffset(const Point& textOffset) noexcept {
        defaultTextOffset = textOffset;
    }
    inline void setDefaultTextPosition(const Point& textPosition) noexcept {
        defaultTextPosition = textPosition;
    }
    inline void setDefaultMargin(const Point& margin) noexcept {
        defaultMargin = margin;
    }

protected:
    inline double x(const double x) const noexcept {
        return fx * (x + dx);
    }
    inline double x(const Point& point) const noexcept {
        return x(point.x);
    }

    inline double y(const double y) const noexcept {
        return fy * (y + dy);
    }
    inline double y(const Point& point) const noexcept {
        return y(point.y);
    }

    inline double ex(const double size) const noexcept {
        return std::abs(size / fx);
    }
    inline double ex() const noexcept {
        return ex(defaultSize);
    }

    inline double ey(const double size) const noexcept {
        return std::abs(size / fy);
    }
    inline double ey() const noexcept {
        return ey(defaultSize);
    }

    inline Point abs(const double x, const double y) const noexcept {
        return Point(Construct::XY, (x / fx) - dx, (y / fy) - dy);
    }

protected:
    double dx;
    double dy;
    double fx;
    double fy;

    Rectangle boundingBox;

    double defaultStroke;
    double defaultSize;
    Color defaultColor;
    Icon defaultIcon;
    XAlignment defaultXAlignment;
    YAlignment defaultYAlignment;
    std::string defaultFont;
    Point defaultTextOffset;
    Point defaultTextPosition;
    Point defaultMargin;

};
