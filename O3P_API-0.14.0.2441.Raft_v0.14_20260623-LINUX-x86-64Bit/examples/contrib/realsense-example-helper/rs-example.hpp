// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015 Intel Corporation. All Rights Reserved.

#pragma once

#include <o3p/o3p.hpp> // Include O3P Cross Platform API

#define GL_SILENCE_DEPRECATION
#define GLFW_INCLUDE_GLU
#include <GLFW/glfw3.h>

#include <algorithm>
#include <cmath>
#include <functional>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <string>

#include "imgui/imgui_impl_glfw.h"
#include "stb/stb_easy_font.h"

#ifndef PI
#define PI 3.14159265358979323846
#define PI_FL 3.141592f
#endif
const float IMU_FRAME_WIDTH = 1280.f;
const float IMU_FRAME_HEIGHT = 720.f;
enum class Priority { high = 0,
                      medium = -1,
                      low = -2 };

/** \brief 3D vector in Euclidean coordinate space */
typedef struct o3p_vector {
    float x, y, z;
} o3p_vector;

typedef struct o3p_vertex {
    float xyz[3];
} o3p_vertex;
//////////////////////////////
// Basic Data Types         //
//////////////////////////////
struct short3 {
    uint16_t x, y, z;
};

struct float3 {
    float x, y, z;
    float3 operator*(float t) {
        return {x * t, y * t, z * t};
    }

    float3 operator-(float t) {
        return {x - t, y - t, z - t};
    }

    void operator*=(float t) {
        x = x * t;
        y = y * t;
        z = z * t;
    }

    void operator=(float3 other) {
        x = other.x;
        y = other.y;
        z = other.z;
    }

    void add(float t1, float t2, float t3) {
        x += t1;
        y += t2;
        z += t3;
    }
};
struct float2 {
    float x, y;
};
struct frame_pixel {
    int frame_idx;
    float2 pixel;
};

struct rect {
    float x, y;
    float w, h;

    // Create new rect within original boundaries with give aspect ration
    rect adjust_ratio(float2 size) const {
        auto H = static_cast<float>(h), W = static_cast<float>(h) * size.x / size.y;
        if (W > w) {
            auto scale = w / W;
            W *= scale;
            H *= scale;
        }

        return {x + (w - W) / 2, y + (h - H) / 2, W, H};
    }
};

struct tile_properties {

    unsigned int x, y; // location of tile in the grid
    unsigned int w, h; // width and height by number of tiles
    Priority priority; // when should the tile be drawn?: high priority is on top of all, medium is a layer under top layer, low is a layer under medium layer
};

// name aliasing the map of pairs<frame, tile_properties>
using frame_and_tile_property = std::pair<o3p::Frame, tile_properties>;
using frames_mosaic = std::map<int, frame_and_tile_property>;

//////////////////////////////
// Simple font loading code //
//////////////////////////////

inline void draw_text(int x, int y, const char *text) {
    std::vector<char> buffer;
    buffer.resize(60000); // ~300 chars
    glEnableClientState(GL_VERTEX_ARRAY);
    glVertexPointer(2, GL_FLOAT, 16, &(buffer[0]));
    glDrawArrays(GL_QUADS,
                 0,
                 4 * stb_easy_font_print((float)x,
                                         (float)(y - 7),
                                         (char *)text,
                                         nullptr,
                                         &(buffer[0]),
                                         int(sizeof(char) * buffer.size())));
    glDisableClientState(GL_VERTEX_ARRAY);
}

void set_viewport(const rect &r) {
    glViewport((int)r.x, (int)r.y, (int)r.w, (int)r.h);
    glLoadIdentity();
    glMatrixMode(GL_PROJECTION);
    glOrtho(0, r.w, r.h, 0, -1, +1);
}

class imu_renderer {
  public:
    void render(o3p::FrameSet *frame, const rect &r) {
        draw_motion(frame, r.adjust_ratio({IMU_FRAME_WIDTH, IMU_FRAME_HEIGHT}));
    }

    GLuint get_gl_handle() { return _gl_handle; }

  private:
    GLuint _gl_handle = 0;

    void draw_motion(o3p::FrameSet *f, const rect &r) {
        if (!_gl_handle)
            glGenTextures(1, &_gl_handle);

        set_viewport(r);
        draw_text(int(0.05f * r.w), int(0.05f * r.h), "IMU Stream");

        auto md = f->getImuFrame();
        auto x = md->acceleration[0];
        auto y = md->acceleration[1];
        auto z = md->acceleration[2];

        glMatrixMode(GL_PROJECTION);
        glPushMatrix();
        glLoadIdentity();

        glOrtho(-2.8, 2.8, -2.4, 2.4, -7, 7);

        glRotatef(25, 1.0f, 0.0f, 0.0f);

        glTranslatef(0, -0.33f, -1.f);

        glRotatef(-135, 0.0f, 1.0f, 0.0f);

        glRotatef(180, 0.0f, 0.0f, 1.0f);
        glRotatef(-90, 0.0f, 1.0f, 0.0f);

        draw_axes(1, 2);

        draw_circle(1, 0, 0, 0, 1, 0);
        draw_circle(0, 1, 0, 0, 0, 1);
        draw_circle(1, 0, 0, 0, 0, 1);

        const auto canvas_size = 230;
        const auto vec_threshold = 0.01f;
        float norm = std::sqrt(x * x + y * y + z * z);
        if (norm < vec_threshold) {
            const auto radius = 0.05;
            static const int circle_points = 100;
            static const float angle = 2.0f * 3.1416f / circle_points;

            glColor3f(1.0f, 1.0f, 1.0f);
            glBegin(GL_POLYGON);
            double angle1 = 0.0;
            glVertex2d(radius * cos(0.0), radius * sin(0.0));
            int i;
            for (i = 0; i < circle_points; i++) {
                glVertex2d(radius * cos(angle1), radius * sin(angle1));
                angle1 += angle;
            }
            glEnd();
        } else {
            auto vectorWidth = 3.f;
            glLineWidth(vectorWidth);
            glBegin(GL_LINES);
            glColor3f(1.0f, 1.0f, 1.0f);
            glVertex3f(0.0f, 0.0f, 0.0f);
            glVertex3f(x / norm, y / norm, z / norm);
            glEnd();

            // Save model and projection matrix for later
            GLfloat model[16];
            glGetFloatv(GL_MODELVIEW_MATRIX, model);
            GLfloat proj[16];
            glGetFloatv(GL_PROJECTION_MATRIX, proj);

            glLoadIdentity();
            glOrtho(-canvas_size, canvas_size, -canvas_size, canvas_size, -1, +1);

            std::ostringstream s1;
            const auto precision = 3;

            glRotatef(180, 1.0f, 0.0f, 0.0f);

            s1 << "(" << std::fixed << std::setprecision(precision) << x << "," << std::fixed << std::setprecision(precision) << y << "," << std::fixed << std::setprecision(precision) << z << ")";
            print_text_in_3d(x, y, z, s1.str().c_str(), false, model, proj, 1 / norm);

            std::ostringstream s2;
            s2 << std::setprecision(precision) << norm;
            print_text_in_3d(x / 2, y / 2, z / 2, s2.str().c_str(), true, model, proj, 1 / norm);
        }
        glMatrixMode(GL_PROJECTION);
        glPopMatrix();
    }

    // IMU drawing helper functions
    void multiply_vector_by_matrix(GLfloat vec[], GLfloat mat[], GLfloat *result) {
        const auto N = 4;
        for (int i = 0; i < N; i++) {
            result[i] = 0;
            for (int j = 0; j < N; j++) {
                result[i] += vec[j] * mat[N * j + i];
            }
        }
        return;
    }

    float2 xyz_to_xy(float x, float y, float z, GLfloat model[], GLfloat proj[], float vec_norm) {
        GLfloat vec[4] = {x, y, z, 0};
        float tmp_result[4];
        float result[4];

        const auto canvas_size = 230;

        multiply_vector_by_matrix(vec, model, tmp_result);
        multiply_vector_by_matrix(tmp_result, proj, result);

        return {canvas_size * vec_norm * result[0], canvas_size * vec_norm * result[1]};
    }

    void print_text_in_3d(float x, float y, float z, const char *text, bool center_text, GLfloat model[], GLfloat proj[], float vec_norm) {
        auto xy = xyz_to_xy(x, y, z, model, proj, vec_norm);
        auto w = (center_text) ? stb_easy_font_width((char *)text) : 0;
        glColor3f(1.0f, 1.0f, 1.0f);
        draw_text((int)(xy.x - w / 2), (int)xy.y, text);
    }

    static void draw_axes(float axis_size = 1.f, float axisWidth = 4.f) {
        // Triangles For X axis
        glBegin(GL_TRIANGLES);
        glColor3f(1.0f, 0.0f, 0.0f);
        glVertex3f(axis_size * 1.1f, 0.f, 0.f);
        glVertex3f(axis_size, -axis_size * 0.05f, 0.f);
        glVertex3f(axis_size, axis_size * 0.05f, 0.f);
        glVertex3f(axis_size * 1.1f, 0.f, 0.f);
        glVertex3f(axis_size, 0.f, -axis_size * 0.05f);
        glVertex3f(axis_size, 0.f, axis_size * 0.05f);
        glEnd();

        // Triangles For Y axis
        glBegin(GL_TRIANGLES);
        glColor3f(0.f, 1.f, 0.f);
        glVertex3f(0.f, axis_size * 1.1f, 0.0f);
        glVertex3f(0.f, axis_size, 0.05f * axis_size);
        glVertex3f(0.f, axis_size, -0.05f * axis_size);
        glVertex3f(0.f, axis_size * 1.1f, 0.0f);
        glVertex3f(0.05f * axis_size, axis_size, 0.f);
        glVertex3f(-0.05f * axis_size, axis_size, 0.f);
        glEnd();

        // Triangles For Z axis
        glBegin(GL_TRIANGLES);
        glColor3f(0.0f, 0.0f, 1.0f);
        glVertex3f(0.0f, 0.0f, 1.1f * axis_size);
        glVertex3f(0.0f, 0.05f * axis_size, 1.0f * axis_size);
        glVertex3f(0.0f, -0.05f * axis_size, 1.0f * axis_size);
        glVertex3f(0.0f, 0.0f, 1.1f * axis_size);
        glVertex3f(0.05f * axis_size, 0.f, 1.0f * axis_size);
        glVertex3f(-0.05f * axis_size, 0.f, 1.0f * axis_size);
        glEnd();

        glLineWidth(axisWidth);

        // Drawing Axis
        glBegin(GL_LINES);
        // X axis - Red
        glColor3f(1.0f, 0.0f, 0.0f);
        glVertex3f(0.0f, 0.0f, 0.0f);
        glVertex3f(axis_size, 0.0f, 0.0f);

        // Y axis - Green
        glColor3f(0.0f, 1.0f, 0.0f);
        glVertex3f(0.0f, 0.0f, 0.0f);
        glVertex3f(0.0f, axis_size, 0.0f);

        // Z axis - Blue
        glColor3f(0.0f, 0.0f, 1.0f);
        glVertex3f(0.0f, 0.0f, 0.0f);
        glVertex3f(0.0f, 0.0f, axis_size);
        glEnd();
    }

    // intensity is grey intensity
    static void draw_circle(float xx, float xy, float xz, float yx, float yy, float yz, float radius = 1.1, float3 center = {0.0, 0.0, 0.0}, float intensity = 0.5f) {
        const auto N = 50;
        glColor3f(intensity, intensity, intensity);
        glLineWidth(2);
        glBegin(GL_LINE_STRIP);

        for (int i = 0; i <= N; i++) {
            const double theta = (2 * PI / N) * i;
            const auto cost = static_cast<float>(cos(theta));
            const auto sint = static_cast<float>(sin(theta));
            glVertex3f(
                center.x + radius * (xx * cost + yx * sint),
                center.y + radius * (xy * cost + yy * sint),
                center.z + radius * (xz * cost + yz * sint));
        }
        glEnd();
    }
};

/// \brief Print flat 2D text over openGl window
struct text_renderer {
    // Provide textual representation only
    void put_text(const std::string &msg, float norm_x_pos, float norm_y_pos, const rect &r) {
        set_viewport(r);
        draw_text(int(norm_x_pos * r.w), int(norm_y_pos * r.h), msg.c_str());
    }
};

////////////////////////
// Image display code //
////////////////////////
/// \brief The texture class
class texture {
  public:
    void upload(const o3p::VideoFrame *frame) {
        if (!_gl_handle)
            glGenTextures(1, &_gl_handle);
        GLenum err = glGetError();
        auto width = 640;
        auto height = 480;

        glBindTexture(GL_TEXTURE_2D, _gl_handle);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, frame->m_data);

        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
        glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
        glBindTexture(GL_TEXTURE_2D, 0);
    }

    void show(const rect &r, float alpha = 1.f) const {
        if (!_gl_handle)
            return;

        set_viewport(r);

        glBindTexture(GL_TEXTURE_2D, _gl_handle);
        glColor4f(1.0f, 1.0f, 1.0f, alpha);
        glEnable(GL_TEXTURE_2D);
        glBegin(GL_QUADS);
        glTexCoord2f(0, 0);
        glVertex2f(0, 0);
        glTexCoord2f(0, 1);
        glVertex2f(0, r.h);
        glTexCoord2f(1, 1);
        glVertex2f(r.w, r.h);
        glTexCoord2f(1, 0);
        glVertex2f(r.w, 0);
        glEnd();
        glDisable(GL_TEXTURE_2D);
        glBindTexture(GL_TEXTURE_2D, 0);
        draw_text(int(0.05f * r.w), int(0.05f * r.h), "ACCEL");
    }

    GLuint get_gl_handle() { return _gl_handle; }

    void render(o3p::FrameSet *frame, const rect &rect, float alpha = 1.f) {
        _imu_render.render(frame, rect.adjust_ratio({IMU_FRAME_WIDTH, IMU_FRAME_HEIGHT}));
    }

  private:
    GLuint _gl_handle = 0;
    int _stream_index{};
    imu_renderer _imu_render;
};

class window {
  public:
    std::function<void(bool)> on_left_mouse = [](bool) {};
    std::function<void(double, double)> on_mouse_scroll = [](double, double) {};
    std::function<void(double, double)> on_mouse_move = [](double, double) {};
    std::function<void(int)> on_key_release = [](int) {};

    window(int width, int height, const char *title)
        : _width(width), _height(height), _canvas_left_top_x(0), _canvas_left_top_y(0), _canvas_width(width), _canvas_height(height) {
        glfwInit();
        win = glfwCreateWindow(width, height, title, nullptr, nullptr);
        if (!win)
            throw std::runtime_error("Could not open OpenGL window, please check your graphic drivers or use the textual SDK tools");
        glfwMakeContextCurrent(win);

        glfwSetWindowUserPointer(win, this);
        glfwSetMouseButtonCallback(win, [](GLFWwindow *w, int button, int action, int mods) {
            if (ImGui::GetCurrentContext() != nullptr) {
                ImGui_ImplGlfw_MouseButtonCallback(w, button, action, mods); // Forward the event to ImGui's GLFW implementation
            }
            auto s = (window *)glfwGetWindowUserPointer(w);
            if (button == 0)
                s->on_left_mouse(action == GLFW_PRESS);
        });

        glfwSetScrollCallback(win, [](GLFWwindow *w, double xoffset, double yoffset) {
            if (ImGui::GetCurrentContext() != nullptr) {
                ImGui_ImplGlfw_ScrollCallback(w, xoffset, yoffset); // Forwards scroll events to ImGui
            }
            auto s = (window *)glfwGetWindowUserPointer(w);
            s->on_mouse_scroll(xoffset, yoffset);
        });

        glfwSetCursorPosCallback(win, [](GLFWwindow *w, double x, double y) {
            if (ImGui::GetCurrentContext() != nullptr) {
                ImGui_ImplGlfw_CursorPosCallback(w, x, y); // Forward the cursor position to ImGui
            }
            auto s = (window *)glfwGetWindowUserPointer(w);
            s->on_mouse_move(x, y);
        });

        glfwSetKeyCallback(win, [](GLFWwindow *w, int key, int scancode, int action, int mods) {
            if (ImGui::GetCurrentContext() != nullptr) {
                ImGui_ImplGlfw_KeyCallback(w, key, scancode, action, mods);
            }
            auto s = (window *)glfwGetWindowUserPointer(w);
            if (0 == action) // on key release
            {
                s->on_key_release(key);
            }
        });
    }

    // another c'tor for adjusting specific frames in specific tiles, this window is NOT resizeable
    window(unsigned width, unsigned height, const char *title, unsigned tiles_in_row, unsigned tiles_in_col, float canvas_width = 0.8f,
           float canvas_height = 0.6f, float canvas_left_top_x = 0.1f, float canvas_left_top_y = 0.075f)
        : _width(width), _height(height), _tiles_in_row(tiles_in_row), _tiles_in_col(tiles_in_col)

    {
        // user input verification for mosaic size, if invalid values were given - set to default
        if (canvas_width < 0 || canvas_width > 1 || canvas_height < 0 || canvas_height > 1 ||
            canvas_left_top_x < 0 || canvas_left_top_x > 1 || canvas_left_top_y < 0 || canvas_left_top_y > 1) {
            std::cout << "Invalid window's size parameter entered, setting to default values" << std::endl;
            canvas_width = 0.8f;
            canvas_height = 0.6f;
            canvas_left_top_x = 0.15f;
            canvas_left_top_y = 0.075f;
        }

        // user input verification for number of tiles in row and column
        if (_tiles_in_row <= 0) {
            _tiles_in_row = 4;
        }
        if (_tiles_in_col <= 0) {
            _tiles_in_col = 2;
        }

        // calculate canvas size
        _canvas_width = int(_width * canvas_width);
        _canvas_height = int(_height * canvas_height);
        _canvas_left_top_x = _width * canvas_left_top_x;
        _canvas_left_top_y = _height * canvas_left_top_y;

        // calculate tile size
        _tile_width_pixels = float(std::floor(_canvas_width / _tiles_in_row));
        _tile_height_pixels = float(std::floor(_canvas_height / _tiles_in_col));

        glfwInit();
        // we don't want to enable resizing the window
        glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);
        win = glfwCreateWindow(width, height, title, nullptr, nullptr);
        if (!win)
            throw std::runtime_error("Could not open OpenGL window, please check your graphic drivers or use the textual SDK tools");
        glfwMakeContextCurrent(win);

        glfwSetWindowUserPointer(win, this);
        glfwSetMouseButtonCallback(win, [](GLFWwindow *w, int button, int action, int mods) {
            if (ImGui::GetCurrentContext() != nullptr) {
                ImGui_ImplGlfw_MouseButtonCallback(w, button, action, mods); // Forward the event to ImGui's GLFW implementation
            }
            auto s = (window *)glfwGetWindowUserPointer(w);
            if (button == 0)
                s->on_left_mouse(action == GLFW_PRESS);
        });

        glfwSetScrollCallback(win, [](GLFWwindow *w, double xoffset, double yoffset) {
            if (ImGui::GetCurrentContext() != nullptr) {
                ImGui_ImplGlfw_ScrollCallback(w, xoffset, yoffset); // Forwards scroll events to ImGui
            }
            auto s = (window *)glfwGetWindowUserPointer(w);
            s->on_mouse_scroll(xoffset, yoffset);
        });

        glfwSetCursorPosCallback(win, [](GLFWwindow *w, double x, double y) {
            if (ImGui::GetCurrentContext() != nullptr) {
                ImGui_ImplGlfw_CursorPosCallback(w, x, y); // Forward the cursor position to ImGui
            }
            auto s = (window *)glfwGetWindowUserPointer(w);
            s->on_mouse_move(x, y);
        });

        glfwSetKeyCallback(win, [](GLFWwindow *w, int key, int scancode, int action, int mods) {
            if (ImGui::GetCurrentContext() != nullptr) {
                ImGui_ImplGlfw_KeyCallback(w, key, scancode, action, mods);
            }
            auto s = (window *)glfwGetWindowUserPointer(w);
            if (0 == action) // on key release
            {
                s->on_key_release(key);
            }
        });
    }

    ~window() {
        glfwDestroyWindow(win);
        glfwTerminate();
    }

    void close() {
        glfwSetWindowShouldClose(win, 1);
    }

    float width() const { return float(_width); }
    float height() const { return float(_height); }

    operator bool() {
        glPopMatrix();
        glfwSwapBuffers(win);

        auto res = !glfwWindowShouldClose(win);

        glfwPollEvents();
        glfwGetFramebufferSize(win, &_width, &_height);

        // Clear the framebuffer
        glClear(GL_COLOR_BUFFER_BIT);
        glViewport(0, 0, _width, _height);

        // Draw the images
        glPushMatrix();
        glfwGetWindowSize(win, &_width, &_height);
        glOrtho(0, _width, _height, 0, -1, +1);

        return res;
    }

    operator GLFWwindow *() { return win; }

  private:
    GLFWwindow *win;
    std::map<int, texture> _textures;
    std::map<int, imu_renderer> _imus;
    text_renderer _main_win;
    int _width, _height;
    float _canvas_left_top_x, _canvas_left_top_y;
    int _canvas_width, _canvas_height;
    unsigned _tiles_in_row, _tiles_in_col;
    float _tile_width_pixels, _tile_height_pixels;

    void render_motion_frame(o3p::FrameSet *f, const rect &r) {
        auto &i = _imus[1];
        i.render(f, r);
    }
};

// Struct to get keys pressed on window
struct window_key_listener {
    int last_key = GLFW_KEY_UNKNOWN;

    window_key_listener(window &win) {
        win.on_key_release = std::bind(&window_key_listener::on_key_release, this, std::placeholders::_1);
    }

    void on_key_release(int key) {
        last_key = key;
    }

    int get_key() {
        int key = last_key;
        last_key = GLFW_KEY_UNKNOWN;
        return key;
    }
};

// Struct for managing rotation of pointcloud view
struct glfw_state {
    glfw_state(float yaw = 15.0, float pitch = 15.0) : yaw(yaw), pitch(pitch), last_x(0.0), last_y(0.0),
                                                       ml(false), offset_x(2.f), offset_y(2.f), tex() {}
    double yaw;
    double pitch;
    double last_x;
    double last_y;
    bool ml;
    float offset_x;
    float offset_y;
    texture tex;
};

void colormap(float t, float &r, float &g, float &b) {
    t = std::min(std::max(t, 0.0f), 1.0f);
    r = std::min(std::max(1.5f - std::abs(4.0f * t - 1.0f), 0.0f), 1.0f);
    g = std::min(std::max(1.5f - std::abs(4.0f * t - 2.0f), 0.0f), 1.0f);
    b = std::min(std::max(1.5f - std::abs(4.0f * t - 3.0f), 0.0f), 1.0f);
}

// Handles all the OpenGL calls needed to display the point cloud
void draw_pointcloud(o3p::Colorizer *colorizer, float width, float height, glfw_state &app_state, o3p::PointcloudFrame *points) {
    // OpenGL commands that prep screen for the pointcloud
    glLoadIdentity();
    glPushAttrib(GL_ALL_ATTRIB_BITS);

    glClearColor(153.f / 255, 153.f / 255, 153.f / 255, 1);
    glClear(GL_DEPTH_BUFFER_BIT);

    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    gluPerspective(60, width / height, 0.01f, 10.0f);

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    gluLookAt(0, 0, 0, 0, 0, 1, 0, -1, 0);

    glTranslatef(0, 0, +0.5f + app_state.offset_y * 0.05f);
    glRotated(app_state.pitch, 1, 0, 0);
    glRotated(app_state.yaw, 0, 1, 0);
    glTranslatef(0, 0, -0.5f);

    glPointSize(width / 640);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, app_state.tex.get_gl_handle());
    float tex_border_color[] = {0.8f, 0.8f, 0.8f, 0.8f};
    glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, tex_border_color);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, 0x812F); // GL_CLAMP_TO_EDGE
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, 0x812F); // GL_CLAMP_TO_EDGE
    glBegin(GL_POINTS);

    /* this segment actually prints the pointcloud */
    auto vertices = std::vector<o3p_vertex>(points->getHeight() * points->getWidth());
    std::vector<float> depthImage(points->getWidth() * points->getHeight());
    for (size_t i = 0; i < depthImage.size(); ++i) {
        depthImage[i] = points->m_data[i * 4 + 2] * 1000; // z only! and in mm
    }

    std::vector<uint8_t> rgbData;
    colorizer->colorize(depthImage.data(), rgbData, points->getWidth(), points->getHeight());
    for (GLsizei i = 0; i < static_cast<GLsizei>(vertices.size()); ++i) {
        vertices[i].xyz[0] = points->m_data[i * 4 + 0];
        vertices[i].xyz[1] = points->m_data[i * 4 + 1];
        vertices[i].xyz[2] = points->m_data[i * 4 + 2];

        float r = static_cast<float>(rgbData[i * 3 + 0]) / 255.0f;
        float g = static_cast<float>(rgbData[i * 3 + 1]) / 255.0f;
        float b = static_cast<float>(rgbData[i * 3 + 2]) / 255.0f;
        glColor3f(r, g, b);
        glVertex3fv(vertices[i].xyz);
    }
    // OpenGL cleanup
    glEnd();
    glPopMatrix();
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glPopAttrib();
}

// Handles all the OpenGL calls needed to display the point cloud with aligned color data
void draw_pointcloud_colored(float width, float height, glfw_state &app_state,
                             o3p::PointcloudFrame *points, o3p::VideoFrame *alignedColor) {
    glLoadIdentity();
    glPushAttrib(GL_ALL_ATTRIB_BITS);

    glClearColor(153.f / 255, 153.f / 255, 153.f / 255, 1);
    glClear(GL_DEPTH_BUFFER_BIT);

    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    gluPerspective(60, width / height, 0.01f, 10.0f);

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    gluLookAt(0, 0, 0, 0, 0, 1, 0, -1, 0);

    glTranslatef(0, 0, +0.5f + app_state.offset_y * 0.05f);
    glRotated(app_state.pitch, 1, 0, 0);
    glRotated(app_state.yaw, 0, 1, 0);
    glTranslatef(0, 0, -0.5f);

    glPointSize(width / 640);
    glEnable(GL_DEPTH_TEST);
    glBegin(GL_POINTS);

    auto w = points->getWidth();
    auto h = points->getHeight();
    auto numPixels = w * h;
    bool isNv12 = (alignedColor->m_bitsPerPixel == 12);
    uint16_t bytesPerPixel = alignedColor->m_bitsPerPixel / 8;

    for (GLsizei i = 0; i < static_cast<GLsizei>(numPixels); ++i) {
        float x = points->m_data[i * 4 + 0];
        float y = points->m_data[i * 4 + 1];
        float z = points->m_data[i * 4 + 2];

        if (z <= 0.0f) {
            continue;
        }

        float rf, gf, bf;
        if (isNv12) {
            size_t yIdx = static_cast<size_t>(i);
            uint16_t col = static_cast<uint16_t>(i % w);
            uint16_t row = static_cast<uint16_t>(i / w);
            size_t uvBase = static_cast<size_t>(w) * h;
            size_t uvIdx = uvBase + (static_cast<size_t>(row) / 2) * w + (static_cast<size_t>(col) / 2) * 2;

            int Y = alignedColor->m_data[yIdx];
            int U = alignedColor->m_data[uvIdx + 0];
            int V = alignedColor->m_data[uvIdx + 1];

            int C = Y - 16;
            int D = U - 128;
            int E = V - 128;

            rf = std::min(std::max((298 * C + 409 * E + 128) >> 8, 0), 255) / 255.0f;
            gf = std::min(std::max((298 * C - 100 * D - 208 * E + 128) >> 8, 0), 255) / 255.0f;
            bf = std::min(std::max((298 * C + 516 * D + 128) >> 8, 0), 255) / 255.0f;
        } else {
            size_t colorIdx = static_cast<size_t>(i) * bytesPerPixel;
            rf = alignedColor->m_data[colorIdx + 0] / 255.0f;
            gf = alignedColor->m_data[colorIdx + 1] / 255.0f;
            bf = alignedColor->m_data[colorIdx + 2] / 255.0f;
        }

        glColor3f(rf, gf, bf);
        glVertex3f(x, y, z);
    }

    glEnd();
    glPopMatrix();
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glPopAttrib();
}

// Registers the state variable and callbacks to allow mouse control of the pointcloud
void register_glfw_callbacks(window &app, glfw_state &app_state) {
    app.on_left_mouse = [&](bool pressed) {
        app_state.ml = pressed;
    };

    app.on_mouse_scroll = [&](double xoffset, double yoffset) {
        app_state.offset_x -= static_cast<float>(xoffset);
        app_state.offset_y -= static_cast<float>(yoffset);
    };

    app.on_mouse_move = [&](double x, double y) {
        if (app_state.ml) {
            app_state.yaw -= (x - app_state.last_x);
            app_state.yaw = std::max(app_state.yaw, -120.0);
            app_state.yaw = std::min(app_state.yaw, +120.0);
            app_state.pitch += (y - app_state.last_y);
            app_state.pitch = std::max(app_state.pitch, -80.0);
            app_state.pitch = std::min(app_state.pitch, +80.0);
        }
        app_state.last_x = x;
        app_state.last_y = y;
    };

    app.on_key_release = [&](int key) {
        if (key == 32) // Escape
        {
            app_state.yaw = app_state.pitch = 0;
            app_state.offset_x = app_state.offset_y = 0.0;
        }
    };
}

void get_screen_resolution(unsigned int &window_width, unsigned int &window_height) {
    glfwInit();
    const GLFWvidmode *mode = glfwGetVideoMode(glfwGetPrimaryMonitor());

    window_width = mode->width;
    window_height = mode->height;
}

void ImGuiSetStyleColors() {
    ImGui::SetNextWindowBgAlpha(0.5f);        // Set the background alpha to 50% transparency
    ImGui::GetStyle().WindowRounding = 10.0f; // Set the window rounding to 10.0f
    ImGui::GetStyle().WindowBorderSize = 0.0f;

    ImGuiStyle &style = ImGui::GetStyle();
    ImVec4 *colors = style.Colors;
    colors[ImGuiCol_WindowBg] = ImVec4(0.1f, 0.1f, 0.1f, 1.0f); // Set the main background color to dark gray

    // Set other window-related colors
    colors[ImGuiCol_TitleBg] = ImVec4(0.286f, 0.298f, 0.549f, 1.0f);
    colors[ImGuiCol_TitleBgActive] = ImVec4(0.286f, 0.298f, 0.549f, 1.0f);

    // Set frame and button colors to gray
    colors[ImGuiCol_FrameBg] = ImVec4(0.2f, 0.2f, 0.2f, 1.0f);
    colors[ImGuiCol_FrameBgHovered] = ImVec4(0.3f, 0.3f, 0.3f, 1.0f);
    colors[ImGuiCol_FrameBgActive] = ImVec4(0.35f, 0.35f, 0.35f, 1.0f);

    colors[ImGuiCol_CheckMark] = ImVec4(0.5f, 0.5f, 0.5f, 1.0f); // Change the color of the checkmark to gray

    colors[ImGuiCol_SliderGrab] = ImVec4(0.4f, 0.4f, 0.4f, 1.0f); // Change the color of the slider grab to gray
}