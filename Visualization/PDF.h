#pragma once

#include <iostream>
#include <vector>
#include <string>

#include <cairo.h>
#include <cairo-pdf.h>

#include "../Helpers/FileSystem/FileSystem.h"

class PDF {

public:
    PDF(const std::string& filename, const int width, const int height) :
        filename(FileSystem::ensureExtension(filename, ".pdf")),
        width(width),
        height(height) {
        open();
    }
    ~PDF() {
        close();
    }

    inline void open() noexcept {
        surface = cairo_pdf_surface_create(filename.c_str(), width, height);
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
        cairo_show_page(context);
    }

protected:
    const std::string filename;
    const int width;
    const int height;

    cairo_surface_t* surface;
    cairo_t* context;

};
