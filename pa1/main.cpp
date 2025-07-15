#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;
#define rad(angle)        (angle / 180.0f * MY_PI)
#define rad_to_angle(rad) (rad * 180.0f / MY_PI)

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

Eigen::Matrix4f get_rotation(Vector3f axis, float angle) {
    float theta_1 = atan2(axis.y(), axis.x());
    Eigen::Matrix4f axis_to_xz_plane = rotate_z(rad_to_angle(theta_1));
    float theta_2 = atan2(sqrt(axis.x() * axis.x() + axis.y() * axis.y()), axis.z());
    Eigen::Matrix4f to_z = rotate_y(rad_to_angle(theta_2));
    return axis_to_xz_plane.inverse() * to_z.inverse() * rotate_z(angle) * to_z * axis_to_xz_plane;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio, float zNear, float zFar) {
    // 使用给定的参数逐个元素地构建透视投影矩阵并返回该矩阵。
    // Create the projection matrix for the given parameters.
    // Then return it.
    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

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

#define CANVAS_SIZE 200
// TODO 理解Viewport变换的实现
int main(int argc, const char **argv) {
    Eigen::Vector3f axis;
    // TODO test
    Eigen::Matrix4f trans = get_rotation();
    return 0;

    float angle_x = 0;
    float angle_y = 0;
    float angle_z = 0;

    rst::rasterizer r(CANVAS_SIZE, CANVAS_SIZE);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    std::vector<Eigen::Vector3f> pos{{0, 1, 0}, {2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}, {0, 1, 3}, {0, 2, 3}, {1, 2, 3}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    float eye_fov = 45;
    float aspect_ratio = 1;
    float zNear = -0.1;
    float zFar = -50;
    // int frame_count = 0;

    while (1) {
        printf("angle_x: %f, angle_y: %f, angle_z: %f\n", angle_x, angle_y, angle_z);
        printf("eye_fov: %f, aspect_ratio: %f, zNear: %f, zFar: %f\n", eye_fov, aspect_ratio, zNear, zFar);
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle_x, angle_y, angle_z));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(eye_fov, aspect_ratio, zNear, zFar));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(CANVAS_SIZE, CANVAS_SIZE, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        // std::cout << "frame count: " << frame_count++ << '\n';

    wait_key:
        key = cv::waitKey(0);

        switch (key) {
            case 'a':
                angle_z += 10;
                break;
            case 'd':
                angle_z -= 10;
                break;
            case 'w':
                angle_y += 10;
                break;
            case 's':
                angle_y -= 10;
                break;
            case 'q':
                angle_x += 10;
                break;
            case 'e':
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
            case 27:
                goto exit;
            default:
                goto wait_key;
        }
    }
exit:
    return 0;
}
