//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>

rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions) {
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices) {
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols) {
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f &v3, float w = 1.0f) {
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}

float cross_2d(Eigen::Vector3f v1, Eigen::Vector3f v2) {
    return ((v1.x() * v2.y()) - (v1.y() * v2.x()));
}

static bool insideTriangle(float x, float y, const Vector3f *_v) {
    // Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    // 在2D平面中判断某点(x,y)是否在三角形内，这里不需要考虑z轴的值！
    Vector3f p(x, y, 0);

    float z0 = cross_2d(_v[1] - _v[0], p - _v[0]);
    float z1 = cross_2d(_v[2] - _v[1], p - _v[1]);
    float z2 = cross_2d(_v[0] - _v[2], p - _v[2]);

    return (z0 > 0 && z1 > 0 && z2 > 0) || (z0 < 0 && z1 < 0 && z2 < 0);
}

static float computeZInterpolate(float x, float y, const Vector4f *v) {
    // Barycentric 3D z interpolate
    float alpha = (x * (v[1].y() - v[2].y()) + (v[2].x() - v[1].x()) * y + v[1].x() * v[2].y() - v[2].x() * v[1].y()) / (v[0].x() * (v[1].y() - v[2].y()) + (v[2].x() - v[1].x()) * v[0].y() + v[1].x() * v[2].y() - v[2].x() * v[1].y());
    float beta = (x * (v[2].y() - v[0].y()) + (v[0].x() - v[2].x()) * y + v[2].x() * v[0].y() - v[0].x() * v[2].y()) / (v[1].x() * (v[2].y() - v[0].y()) + (v[0].x() - v[2].x()) * v[1].y() + v[2].x() * v[0].y() - v[0].x() * v[2].y());
    float gamma = 1 - alpha - beta;

    // 框架代码有问题，正确的代码见pa3
    // 这里的透视修整算法是对的，但v[].w()取出来的都是1，因为Triangle里面根本没存w的值，就相当于没做修正。
    float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
    z_interpolated *= w_reciprocal;

    return z_interpolated;
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type) {
    auto &buf = pos_buf[pos_buffer.pos_id];
    auto &ind = ind_buf[ind_buffer.ind_id];
    auto &col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    size_t counter = 0;
    Eigen::Matrix4f mvp = projection * view * model;
    for (auto &i : ind) {
        Triangle t;
        Eigen::Vector4f v[] = {
            mvp * to_vec4(buf[i[0]], 1.0f),
            mvp * to_vec4(buf[i[1]], 1.0f),
            mvp * to_vec4(buf[i[2]], 1.0f)
        };
        // Homogeneous division
        for (auto &vec : v) {
            // 框架代码有问题，正确的代码见pa3
            // 这里不应该改变w值
            vec /= vec.w();
            vec.z() *= -1;  // 勾八框架非要把z轴看成正数，我只好在这里给你翻转了
        }
        // Viewport transformation
        for (auto &vert : v) {
            vert.x() = 0.5 * width * (vert.x() + 1.0);
            vert.y() = 0.5 * height * (vert.y() + 1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i) {
            t.setVertex(i, v[i].head<3>());
        }

        auto color = col[counter++];
        t.setColor(color[0], color[1], color[2]);

        rasterize_triangle(t);
    }

#ifdef SSAA_ENABLE
    for (size_t i = 0; i < frame_buf.size(); i++) {
        frame_buf[i] = (SSAA_frame_buf[4 * i] + SSAA_frame_buf[4 * i + 1] + SSAA_frame_buf[4 * i + 2] + SSAA_frame_buf[4 * i + 3]) / 4;
    }
#endif
}

// Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle &t) {
    Eigen::Vector4f *v = t.toVector4().data();

    // Find out the bounding box of current triangle.
    // iterate through the pixel and find if the current pixel is inside the triangle

    float x_min = std::min(v[0][0], std::min(v[1][0], v[2][0]));
    float x_max = std::max(v[0][0], std::max(v[1][0], v[2][0]));
    float y_min = std::min(v[0][1], std::min(v[1][1], v[2][1]));
    float y_max = std::max(v[0][1], std::max(v[1][1], v[2][1]));

    x_min = std::max(std::floor(x_min), 0.0f);
    x_max = std::min(std::ceil(x_max), (float)(width - 1));
    y_min = std::max(std::floor(y_min), 0.0f);
    y_max = std::min(std::ceil(y_max), (float)height - 1);

#ifdef SSAA_ENABLE
    for (int x = x_min; x <= x_max; x++) {
        for (int y = y_min; y <= y_max; y++) {
            int ind = get_index(x, y) * 4;
            float xs[] = {0.25, 0.25, 0.75, 0.75};
            float ys[] = {0.25, 0.75, 0.25, 0.75};
            for (size_t i = 0; i < 4; i++) {
                float xx = x + xs[i];
                float yy = y + ys[i];
                if (insideTriangle(xx, yy, t.v)) {
                    float z_interpolated = computeZInterpolate(xx, yy, v);
                    if (z_interpolated < depth_buf[ind]) {
                        depth_buf[ind] = z_interpolated;
                        SSAA_frame_buf[ind] = t.color;
                    }
                }
                ind++;
            }
        }
    }
#else
    for (int x = x_min; x <= x_max; x++) {
        for (int y = y_min; y <= y_max; y++) {
            if (insideTriangle(x + 0.5, y + 0.5, t.v)) {
                // If so, use the following code to get the interpolated z value.
                float z_interpolated = computeZInterpolate(x, y, v);
                // set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
                int ind = get_index(x, y);
                if (z_interpolated < depth_buf[ind]) {
                    depth_buf[ind] = z_interpolated;
                    frame_buf[ind] = t.color;
                }
            }
        }
    }
#endif
}

void rst::rasterizer::set_model(const Eigen::Matrix4f &m) {
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f &v) {
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f &p) {
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff) {
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color) {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
#ifdef SSAA_ENABLE
        std::fill(SSAA_frame_buf.begin(), SSAA_frame_buf.end(), Eigen::Vector3f{0, 0, 0});
#endif
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth) {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h) {
    frame_buf.resize(w * h);
#ifdef SSAA_ENABLE
    SSAA_frame_buf.resize(w * h * 4);
    depth_buf.resize(w * h * 4);
#else
    depth_buf.resize(w * h);
#endif
}

int rst::rasterizer::get_index(int x, int y) {
    return (height - 1 - y) * width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f &point, const Eigen::Vector3f &color) {
    // old index: auto ind = point.y() + point.x() * width;
    auto ind = (height - 1 - point.y()) * width + point.x();
    frame_buf[ind] = color;
}
