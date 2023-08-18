#pragma once

#include <iostream>
#include <vector>
#include <string>

#include <cairo.h>
#include <cairo-svg.h>

#include "../Helpers/FileSystem/FileSystem.h"

class SVG {

public:
    SVG(const std::string& filename, const int width, const int height) :
        filename(FileSystem::ensureExtension(filename, ".svg")),
        width(width),
        height(height),
        filenameBase(filename),
        page(1) {
        open();
    }
    ~SVG() {
        close();
    }

    inline void open() noexcept {
        surface = cairo_svg_surface_create(filename.c_str(), width, height);
        context = cairo_create(surface);
    }

    inline void close() noexcept {
        cairo_surface_destroy(surface);
        cairo_destroy(context);
        surface = nullptr;
        context = nullptr;
    }

    inline bool isOpen() const noexcept {
        return surface != nullptr && context != nullptr;
    }

    inline void newPage() noexcept {
        filename = FileSystem::ensureExtension(filenameBase + "_" + std::to_string(page), ".svg");
        close();
        page++;
        filename = FileSystem::ensureExtension(filenameBase + "_" + std::to_string(page), ".svg");
        open();
    }

protected:
    std::string filename;
    const int width;
    const int height;

    cairo_surface_t* surface;
    cairo_t* context;

    std::string filenameBase;
    int page;

};
