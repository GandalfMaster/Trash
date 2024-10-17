// clang-format off
#include <iostream>
#include <opencv2/opencv.hpp>
#include "rasterizer.hpp"
#include "global.hpp"
#include "Triangle.hpp"
constexpr double MY_PI = 3.1415926;

//视图矩阵，世界空间->摄像机空间
Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();
    //平移矩阵，将摄像机位置到原点。
    Eigen::Matrix4f translate;
    translate << 1,0,0,-eye_pos[0],
                 0,1,0,-eye_pos[1],
                 0,0,1,-eye_pos[2],
                 0,0,0,1;

    view = translate*view;

    return view;
}

//模型矩阵，旋转物体
Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
    float rotation_angle_radian = rotation_angle * MY_PI / 180;
    //下面是x,y轴的旋转矩阵 
    model(0, 0) = cos(rotation_angle_radian);
    model(0, 1) = -sin(rotation_angle_radian);
    model(1, 0) = sin(rotation_angle_radian);
    model(1, 1) = cos(rotation_angle_radian);

    return model;
}

//投影矩阵函数，得到透视投影的二维矩阵
Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio, float zNear, float zFar)
{
    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();
    //zNear = -zNear;
    //zFar = -zFar;
    //计算透视投影上下左右边界
    float t = tan((eye_fov * MY_PI / 180) / 2) * fabs(zNear);
    float r = aspect_ratio * t;
    float l = -r;
    float b = -t;
    //构建正交投影矩阵
    Eigen::Matrix4f translate = Eigen::Matrix4f::Identity();
    translate(0, 3) = -(r + l) / 2;
    translate(1, 3) = -(t + b) / 2;
    translate(2, 3) = -(zNear + zFar) / 2;

    Eigen::Matrix4f scale = Eigen::Matrix4f::Identity();
    scale(0, 0) = 2 / (r - l);
    scale(1, 1) = 2 / (t - b);
    scale(2, 2) = 2 / (zNear - zFar);

    Eigen::Matrix4f ortho = scale * translate;

    //透视投影到正交投影的转换矩阵
    Eigen::Matrix4f persp2ortho = Eigen::Matrix4f::Zero();

    persp2ortho(0, 0) = zNear;
    persp2ortho(1, 1) = zNear;
    persp2ortho(2, 2) = zNear + zFar;
    persp2ortho(2, 3) = -zNear * zFar;
    persp2ortho(3, 2) = 1;
    projection = ortho * persp2ortho;

    return projection;
}

int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc == 2)
    {
        command_line = true;
        filename = std::string(argv[1]);
    }

    rst::rasterizer r(700, 700);

    //定义摄像机的位置
    Eigen::Vector3f eye_pos = {0,0,10};


    std::vector<Eigen::Vector3f> pos
    {
        //三角形1
            {2, 0, -2},
            {0, 2, -2},
            {-2, 0, -2},
        //三角形2
            {3.5, -1, -5},
            {2.5, 1.5, -5},
            {-1, 0.5, -5}
    };

    std::vector<Eigen::Vector3i> ind    //三角形1和2的各顶点索引
    {
            {0, 1, 2},
            {3, 4, 5}
    };

    std::vector<Eigen::Vector3f> cols
    {
        //三角形1顶点颜色
            {217.0, 238.0, 185.0},
            {217.0, 238.0, 185.0},
            {217.0, 238.0, 185.0},
        //三角形2顶点颜色
            {185.0, 217.0, 238.0},
            {185.0, 217.0, 238.0},
            {185.0, 217.0, 238.0}
    };

    //加载位置，索引，颜色的缓冲区
    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);
    auto col_id = r.load_colors(cols);

    int key = 0;
    int frame_count = 0;

    //命令行模式，生成渲染后的图片
    if (command_line)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, col_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

        cv::imwrite(filename, image);

        return 0;
    }

    //主循环，窗口实时显示渲染结果
    while(key != 27)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(30));//设置旋转角度
        r.set_view(get_view_matrix(eye_pos));//设置视图矩阵
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));//设置投影矩阵
        //绘制三角形
        r.draw(pos_id, ind_id, col_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);//转换为8位无符号图像
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);//转换颜色格式
        cv::imshow("image", image);//显示图像
        key = cv::waitKey(10);//等待按键

        std::cout << "frame count: " << frame_count++ << '\n';
    }

    return 0;
}
// clang-format on