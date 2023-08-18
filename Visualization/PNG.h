#pragma once

#include <iostream>
#include <vector>
#include <string>

#include <cairo.h>

#include "../Helpers/FileSystem/FileSystem.h"

class PNG {

public:
    PNG(const std::string& filename, const int width, const int height) :
        filename(FileSystem::ensureExtension(filename, ".png")),
        width(width),
        height(height),
        filenameBase(filename),
        page(1) {
        open();
    }
    ~PNG() {
        close();
    }

    inline void open() noexcept {
        surface = cairo_image_surface_create(CAIRO_FORMAT_ARGB32, width, height);
        context = cairo_create(surface);
    }

    inline void close() noexcept {
        cairo_surface_write_to_png(surface, filename.c_str());
        cairo_surface_destroy(surface);
        cairo_destroy(context);
        surface = nullptr;
        context = nullptr;
    }

    inline bool isOpen() const noexcept {
        return surface != nullptr && context != nullptr;
    }

    inline void newPage() noexcept {
        filename = FileSystem::ensureExtension(filenameBase + "_" + std::to_string(page), ".png");
        close();
        page++;
        filename = FileSystem::ensureExtension(filenameBase + "_" + std::to_string(page), ".png");
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
