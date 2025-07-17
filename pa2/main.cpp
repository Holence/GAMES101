#include <iostream>
#include <opencv2/opencv.hpp>
#include "rasterizer.hpp"
#include "global.hpp"
#include "Triangle.hpp"

constexpr double MY_PI = 3.1415926;
#define rad(angle) (angle / 180.0f * MY_PI)

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos) {
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0],
        0, 1, 0, -eye_pos[1],
        0, 0, 1, -eye_pos[2],
        0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f rotate_x(float rotation_angle) {
    float rad = rad(rotation_angle);
    Eigen::Matrix4f rotate;
    rotate << 1, 0, 0, 0,
        0, cos(rad), -sin(rad), 0,
        0, sin(rad), cos(rad), 0,
        0, 0, 0, 1;
    return rotate;
}

Eigen::Matrix4f rotate_y(float rotation_angle) {
    float rad = rad(rotation_angle);
    Eigen::Matrix4f rotate;
    rotate << cos(rad), 0, -sin(rad), 0,
        0, 1, 0, 0,
        sin(rad), 0, cos(rad), 0,
        0, 0, 0, 1;
    return rotate;
}

Eigen::Matrix4f rotate_z(float rotation_angle) {
    float rad = rad(rotation_angle);
    Eigen::Matrix4f rotate;
    rotate << cos(rad), -sin(rad), 0, 0,
        sin(rad), cos(rad), 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;
    return rotate;
}

Eigen::Matrix4f get_model_matrix(float angle_x, float angle_y, float angle_z) {
    return rotate_x(angle_x) * rotate_y(angle_y) * rotate_z(angle_z);
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio, float zNear, float zFar) {
    Eigen::Matrix4f projection;

    float n = zNear;
    float f = zFar;
    float t = abs(n) * tan(rad(eye_fov / 2));
    float b = -t;
    float r = aspect_ratio * t;
    float l = -r;

    Eigen::Matrix4f pers;
    pers << n, 0, 0, 0,
        0, n, 0, 0,
        0, 0, n + f, -f * n,
        0, 0, 1, 0;

    Eigen::Matrix4f orth;
    orth << 2 / (r - l), 0, 0, -(r + l) / (r - l),
        0, 2 / (t - b), 0, -(t + b) / (t - b),
        0, 0, 2 / (n - f), -(n + f) / (n - f),
        0, 0, 0, 1;

    projection = orth * pers;

    return projection;
}

#define CANVAS_SIZE 500
int main(int argc, const char **argv) {
    float angle_x = 0;
    float angle_y = 0;
    float angle_z = 0;

    rst::rasterizer r(CANVAS_SIZE, CANVAS_SIZE);

    Eigen::Vector3f eye_pos = {0, 0, 7.5};

    // std::vector<Eigen::Vector3f> pos{
    //     {2, 0, -2},
    //     {0, 2, -2},
    //     {-2, 0, -2},
    //     {3.5, -1, -5},
    //     {2.5, 1.5, -5},
    //     {-1, 0.5, -5}
    // };

    // std::vector<Eigen::Vector3i> ind{
    //     {0, 1, 2},
    //     {3, 4, 5}
    // };

    // std::vector<Eigen::Vector3f> cols{
    //     {217.0, 238.0, 185.0},
    //     {185.0, 217.0, 238.0}
    // };

    // Three Overlapping Triangles
#define s3 (sqrt(3))
    std::vector<Eigen::Vector3f> pos{
        {2, -s3, -2},
        {-1, 2 * s3, -1},
        {0, 2 * s3, -1},

        {-3, 0, -2},
        {3, 0, -1},
        {2.5, s3 / 2, -1},

        {1, 2 * s3, -2},
        {-2, -s3, -1},
        {-2.5, -s3 / 2, -1},
    };
    for (auto &v : pos) {
        v.y() -= 1 / s3;
    }

    std::vector<Eigen::Vector3i> ind{
        {0, 1, 2},
        {3, 4, 5},
        {6, 7, 8},
    };

    std::vector<Eigen::Vector3f> cols{
        {217.0, 238.0, 185.0},
        {185.0, 217.0, 238.0},
        {238.0, 185.0, 217.0}
    };

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);
    auto col_id = r.load_colors(cols);

    int key = 0;
    float eye_fov = 45;
    float aspect_ratio = 1;
    float zNear = -0.1;
    float zFar = -50;

    while (1) {
        printf("eye_pos: (%f, %f, %f)\n", eye_pos.x(), eye_pos.y(), eye_pos.z());
        printf("angle_x: %f, angle_y: %f, angle_z: %f\n", angle_x, angle_y, angle_z);
        printf("eye_fov: %f, aspect_ratio: %f, zNear: %f, zFar: %f\n\n", eye_fov, aspect_ratio, zNear, zFar);
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle_x, angle_y, angle_z));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(eye_fov, aspect_ratio, zNear, zFar));

        r.draw(pos_id, ind_id, col_id, rst::Primitive::Triangle);

        cv::Mat image(CANVAS_SIZE, CANVAS_SIZE, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
        cv::imshow("image", image);

    wait_key:
        key = cv::waitKey(0);
        switch (key) {
            case 'a':
                angle_z += 10;
                break;
            case 'd':
                angle_z -= 10;
                break;
            case 'e':
                angle_y += 10;
                break;
            case 'q':
                angle_y -= 10;
                break;
            case 's':
                angle_x += 10;
                break;
            case 'w':
                angle_x -= 10;
                break;
            case '1':
                eye_fov -= 1;
                break;
            case '2':
                eye_fov += 1;
                break;
            case '3':
                aspect_ratio -= 0.1;
                break;
            case '4':
                aspect_ratio += 0.1;
                break;
            case '5':
                zNear -= 0.1;
                break;
            case '6':
                zNear += 0.1;
                break;
            case '7':
                zFar -= 5;
                break;
            case '8':
                zFar += 5;
                break;
            case 81:  // left key
                eye_pos.x() -= 1;
                break;
            case 83:  // right key
                eye_pos.x() += 1;
                break;
            case 84:  // down key
                eye_pos.y() -= 1;
                break;
            case 82:  // up key
                eye_pos.y() += 1;
                break;
            case '-':
                eye_pos.z() -= 1;
                break;
            case '=':
                eye_pos.z() += 1;
                break;
            case 27:
                goto exit;
            default:
                goto wait_key;
        }
    }
exit:
    return 0;
}
