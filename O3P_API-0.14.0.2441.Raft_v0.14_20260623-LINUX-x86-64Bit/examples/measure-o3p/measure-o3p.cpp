/****************************************************************************\
* Copyright (C) 2026 pmdtechnologies gmbh
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
* THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
* BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
* GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
* OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
\****************************************************************************/

#include "realsense-example-helper/rs-example.hpp"
#include <iostream>
#include <o3p/o3p.hpp>
#include <sstream>

#define _USE_MATH_DEFINES
#include <math.h>

using pixel = std::pair<int, int>;

// Distance 3D is used to calculate real 3D distance between two pixels
// using the pointcloud frame which provides cartesian (X,Y,Z) coordinates
float dist_3d(o3p::PointcloudFrame *pc, pixel u, pixel v);

// Toggle helper class will be used to render the two buttons
// controlling the edges of our ruler
struct toggle {
    toggle() : x(0.f), y(0.f) {}
    toggle(float xl, float yl)
        : x(std::min(std::max(xl, 0.f), 1.f)),
          y(std::min(std::max(yl, 0.f), 1.f)) {}

    // Move from [0,1] space to pixel space using the given frame dimensions
    pixel get_pixel(uint16_t width, uint16_t height) const {
        int px = static_cast<int>(x * width);
        int py = static_cast<int>(y * height);
        return {px, py};
    }

    void render(const window &app) {
        glColor4f(0.f, 0.0f, 0.0f, 0.2f);
        render_circle(app, 10);
        render_circle(app, 8);
        glColor4f(1.f, 0.9f, 1.0f, 1.f);
        render_circle(app, 6);
    }

    void render_circle(const window &app, float r) {
        const float segments = 16.0f;
        glBegin(GL_TRIANGLE_STRIP);
        for (auto i = 0; i <= static_cast<int>(segments); i++) {
            auto t = 2 * PI_FL * static_cast<float>(i) / segments;

            glVertex2f(x * app.width() + cos(t) * r,
                       y * app.height() + sin(t) * r);

            glVertex2f(x * app.width(),
                       y * app.height());
        }
        glEnd();
    }

    // This helper function is used to find the button closest to the mouse cursor.
    // Since we are only comparing this distance, sqrt can be safely skipped
    float dist_2d(const toggle &other) const {
        return static_cast<float>(pow(x - other.x, 2) + pow(y - other.y, 2));
    }

    float x;
    float y;
    bool selected = false;
};

// Application state shared between the main-thread and GLFW events
struct state {
    bool mouse_down = false;
    toggle ruler_start;
    toggle ruler_end;
};

// Helper function to register to UI events
void register_glfw_callbacks(window &app, state &app_state);

// depthRenderer: colorizes a DepthFrame and displays it as a textured quad
// within the given screen region, using the rect type from rs-example.hpp.
class depthRenderer {
    GLuint _gl_handle = 0;
    o3p::Colorizer _colorizer;

  public:
    void render(o3p::DepthFrame *depth, const rect &r) {
        // Colorize depth data (uint16_t mm) to RGB
        std::vector<uint8_t> rgb_data;
        _colorizer.colorize(depth->m_data, rgb_data,
                            depth->getWidth(), depth->getHeight());

        // Upload to GL texture
        if (!_gl_handle)
            glGenTextures(1, &_gl_handle);
        glBindTexture(GL_TEXTURE_2D, _gl_handle);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB,
                     depth->getWidth(), depth->getHeight(),
                     0, GL_RGB, GL_UNSIGNED_BYTE, rgb_data.data());
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
        glBindTexture(GL_TEXTURE_2D, 0);

        // Draw textured quad covering the given rect
        glBindTexture(GL_TEXTURE_2D, _gl_handle);
        glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
        glEnable(GL_TEXTURE_2D);
        glBegin(GL_QUADS);
        glTexCoord2f(0, 0);
        glVertex2f(r.x, r.y);
        glTexCoord2f(0, 1);
        glVertex2f(r.x, r.y + r.h);
        glTexCoord2f(1, 1);
        glVertex2f(r.x + r.w, r.y + r.h);
        glTexCoord2f(1, 0);
        glVertex2f(r.x + r.w, r.y);
        glEnd();
        glDisable(GL_TEXTURE_2D);
        glBindTexture(GL_TEXTURE_2D, 0);
    }

    ~depthRenderer() {
        if (_gl_handle)
            glDeleteTextures(1, &_gl_handle);
    }
};

// irRenderer: overlays an 8-bit grayscale InfraredFrame on top of depth.
// The alpha parameter controls the blend strength (0 = invisible, 1 = opaque).
class irRenderer {
    GLuint _gl_handle = 0;

  public:
    void render(o3p::VideoFrame *ir, const rect &r, float alpha = 0.45f) {
        if (!_gl_handle)
            glGenTextures(1, &_gl_handle);
        glBindTexture(GL_TEXTURE_2D, _gl_handle);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE,
                     ir->getWidth(), ir->getHeight(),
                     0, GL_LUMINANCE, GL_UNSIGNED_BYTE, ir->m_data);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
        glBindTexture(GL_TEXTURE_2D, 0);

        glBindTexture(GL_TEXTURE_2D, _gl_handle);
        glColor4f(1.0f, 1.0f, 1.0f, alpha);
        glEnable(GL_TEXTURE_2D);
        glBegin(GL_QUADS);
        glTexCoord2f(0, 0);
        glVertex2f(r.x, r.y);
        glTexCoord2f(0, 1);
        glVertex2f(r.x, r.y + r.h);
        glTexCoord2f(1, 1);
        glVertex2f(r.x + r.w, r.y + r.h);
        glTexCoord2f(1, 0);
        glVertex2f(r.x + r.w, r.y);
        glEnd();
        glDisable(GL_TEXTURE_2D);
        glBindTexture(GL_TEXTURE_2D, 0);
    }

    ~irRenderer() {
        if (_gl_handle)
            glDeleteTextures(1, &_gl_handle);
    }
};

// Draw a line segment between two 2D screen points with the given line width
void draw_line(float x0, float y0, float x1, float y1, int width) {
    glLineWidth(static_cast<float>(width));
    glBegin(GL_LINES);
    glVertex2f(x0, y0);
    glVertex2f(x1, y1);
    glEnd();
    glLineWidth(1.f);
}

// Renders the ruler line and the distance label between the two ruler endpoints.
// draw_text() is provided by rs-example.hpp via stb_easy_font.
void render_simple_distance(o3p::DepthFrame *depth,
                            o3p::PointcloudFrame *pc,
                            const state &s,
                            const window &app) {
    uint16_t w = depth->getWidth();
    uint16_t h = depth->getHeight();

    pixel center;

    glColor4f(0.f, 0.0f, 0.0f, 0.2f);
    draw_line(s.ruler_start.x * app.width(),
              s.ruler_start.y * app.height(),
              s.ruler_end.x * app.width(),
              s.ruler_end.y * app.height(), 9);

    glColor4f(0.f, 0.0f, 0.0f, 0.3f);
    draw_line(s.ruler_start.x * app.width(),
              s.ruler_start.y * app.height(),
              s.ruler_end.x * app.width(),
              s.ruler_end.y * app.height(), 7);

    glColor4f(1.f, 1.0f, 1.0f, 1.f);
    draw_line(s.ruler_start.x * app.width(),
              s.ruler_start.y * app.height(),
              s.ruler_end.x * app.width(),
              s.ruler_end.y * app.height(), 3);

    auto from_pixel = s.ruler_start.get_pixel(w, h);
    auto to_pixel = s.ruler_end.get_pixel(w, h);
    float air_dist = dist_3d(pc, from_pixel, to_pixel);

    center.first = (from_pixel.first + to_pixel.first) / 2;
    center.second = (from_pixel.second + to_pixel.second) / 2;

    std::stringstream ss;
    ss << int(air_dist * 100) << " cm";
    auto str = ss.str();
    auto x = (float(center.first) / w) * app.width() + 15;
    auto y = (float(center.second) / h) * app.height() + 15;

    auto label_w = static_cast<float>(stb_easy_font_width((char *)str.c_str()));

    // Draw dark background for the text label
    glColor4f(0.f, 0.f, 0.f, 0.4f);
    glBegin(GL_TRIANGLES);
    glVertex2f(x - 3, y - 10);
    glVertex2f(x + label_w + 2, y - 10);
    glVertex2f(x + label_w + 2, y + 2);
    glVertex2f(x + label_w + 2, y + 2);
    glVertex2f(x - 3, y + 2);
    glVertex2f(x - 3, y - 10);
    glEnd();

    // Draw white text label
    glColor4f(1.f, 1.f, 1.f, 1.f);
    draw_text(static_cast<int>(x), static_cast<int>(y), str.c_str());
}

int main(int argc, char *argv[]) {
    const auto cameras = o3p::Context::getAvailableCameras();
    if (cameras.size() > 0) {
        try {
            // Create a Pipeline - this serves as a top-level API for streaming and processing frames
            o3p::Pipeline pipe;
            o3p::PipelineProfile profile;
            o3p::PipelineConfig config;

            // If a serial number is provided on the command line, use that device
            if (argc > 1)
                config.enableDevice(argv[1]);

            try {
                profile = pipe.init(config);
            } catch (std::exception &e) {
                std::cerr << "Error initializing camera : " << e.what() << std::endl;
                return EXIT_FAILURE;
            }

            auto streams = profile.getStreams();

            std::cout << streams.size() << " streams" << std::endl;

            // Find depth stream dimensions to size the window
            uint16_t depth_width = 224;
            uint16_t depth_height = 172;
            for (auto &stream : streams) {
                if (stream->streamType() == o3p::O3P_STREAM_DEPTH) {
                    auto *vs = reinterpret_cast<o3p::VideoStream *>(stream.get());
                    depth_width = vs->width();
                    depth_height = vs->height();
                    break;
                }
            }

            // Initialize window for rendering
            window app(depth_width * 5, depth_height * 5, "O3P Measure Example");

            // Define application state and position the ruler buttons
            state app_state;
            app_state.ruler_start = {0.45f, 0.5f};
            app_state.ruler_end = {0.55f, 0.5f};
            register_glfw_callbacks(app, app_state);

            // Declare renderers
            depthRenderer depth_display;
            irRenderer ir_display;

            pipe.start();

            while (app) {
                // Block program until frames arrive
                o3p::FrameSet frames = pipe.waitForFrames();

                auto *ir = frames.getInfraredFrame();
                auto *depth = frames.getDepthFrame();
                auto *pc = frames.getPointCloudFrame();

                const rect full_window{0.f, 0.f, (float)app.width(), (float)app.height()};

                glEnable(GL_BLEND);
                glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

                // Render colorized depth as the base layer
                depth_display.render(depth, full_window);

                // Overlay IR (grayscale, semi-transparent) on top of depth
                if (ir->getWidth() > 0 && ir->getHeight() > 0)
                    ir_display.render(ir, full_window);

                // Render the ruler line and distance label
                render_simple_distance(depth, pc, app_state, app);

                // Render the ruler endpoint toggles
                app_state.ruler_start.render(app);
                app_state.ruler_end.render(app);

                glColor3f(1.f, 1.f, 1.f);
                glDisable(GL_BLEND);
            }

            // Stop the pipeline
            pipe.stop();

        } catch (std::exception &e) {
            std::cerr << "Error : " << e.what() << std::endl;
            return EXIT_FAILURE;
        }
    } else {
        std::cerr << "Error finding a camera" << std::endl;
        return EXIT_FAILURE;
    }
}

float dist_3d(o3p::PointcloudFrame *pc, pixel u, pixel v) {
    // The PointcloudFrame provides pre-computed cartesian (X, Y, Z) in meters
    // for each pixel, so no manual deprojection is needed.
    float ux = pc->getX(static_cast<uint16_t>(u.first), static_cast<uint16_t>(u.second));
    float uy = pc->getY(static_cast<uint16_t>(u.first), static_cast<uint16_t>(u.second));
    float uz = pc->getZ(static_cast<uint16_t>(u.first), static_cast<uint16_t>(u.second));

    float vx = pc->getX(static_cast<uint16_t>(v.first), static_cast<uint16_t>(v.second));
    float vy = pc->getY(static_cast<uint16_t>(v.first), static_cast<uint16_t>(v.second));
    float vz = pc->getZ(static_cast<uint16_t>(v.first), static_cast<uint16_t>(v.second));

    // Calculate Euclidean distance between the two 3D points
    return sqrt(pow(ux - vx, 2.f) +
                pow(uy - vy, 2.f) +
                pow(uz - vz, 2.f));
}

// Implement drag & drop behavior for the ruler endpoint buttons
void register_glfw_callbacks(window &app, state &app_state) {
    app.on_left_mouse = [&](bool pressed) {
        app_state.mouse_down = pressed;
    };

    app.on_mouse_move = [&](double x, double y) {
        toggle cursor{float(x) / app.width(), float(y) / app.height()};
        std::vector<toggle *> toggles{
            &app_state.ruler_start,
            &app_state.ruler_end};

        if (app_state.mouse_down) {
            toggle *best = toggles.front();
            for (auto &&t : toggles) {
                if (t->dist_2d(cursor) < best->dist_2d(cursor))
                    best = t;
            }
            best->selected = true;
        } else {
            for (auto &&t : toggles)
                t->selected = false;
        }

        for (auto &&t : toggles) {
            if (t->selected)
                *t = cursor;
        }
    };
}
