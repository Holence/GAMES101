#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

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

Eigen::Matrix4f get_model_matrix(float rotation_angle) {
    // 逐个元素地构建模型变换矩阵并返回该矩阵。在此函数中，你只需要实现三维中绕 z 轴旋转的变换矩阵，而不用处理平移与缩放
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.

    Eigen::Matrix4f model;
    // cos, -sin, 0, 0
    // sin,  cos, 0, 0
    //   0,    0, 1, 0
    //   0,    0, 0, 1
    float rad = rad(rotation_angle);
    model << cos(rad), -sin(rad), 0, 0,
        sin(rad), cos(rad), 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;

    return model;
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

// TODO 理解Viewport变换的实现
int main(int argc, const char **argv) {
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]);  // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        } else
            return 0;
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    float eye_fov = 45;
    float aspect_ratio = 1;
    float zNear = -0.1;
    float zFar = -50;
    // int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(eye_fov, aspect_ratio, zNear, zFar));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (1) {
        printf("angle: %f, eye_fov: %f, aspect_ratio: %f, zNear: %f, zFar: %f\n", angle, eye_fov, aspect_ratio, zNear, zFar);
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(eye_fov, aspect_ratio, zNear, zFar));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        // std::cout << "frame count: " << frame_count++ << '\n';

    wait_key:
        key = cv::waitKey(0);

        switch (key) {
            case 'a':
                angle += 10;
                break;
            case 'd':
                angle -= 10;
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
