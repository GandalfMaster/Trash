// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>
#include <tuple>
#include <D:\CG\eigen-3.4.0\Eigen\Eigen>

rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}


static bool insideTriangle(float x, float y, const Vector3f* _v)
{
    
    Vector3f point0 = _v[0];
    Vector3f point1 = _v[1];
    Vector3f point2 = _v[2];

    
    Vector3f vector0 = Vector3f(x - point0.x(), y - point0.y(), 0);
    Vector3f vector1 = Vector3f(x - point1.x(), y - point1.y(), 0);
    Vector3f vector2 = Vector3f(x - point2.x(), y - point2.y(), 0);

    
    Vector3f edge0 = point1 - point0;
    Vector3f edge1 = point2 - point1;
    Vector3f edge2 = point0 - point2;
    
    auto cross0 = edge0.cross(vector0);
    auto cross1 = edge1.cross(vector1);
    auto cross2 = edge2.cross(vector2);
    
    if (cross0.z() > 0 && cross1.z() > 0 && cross2.z() > 0)return true;
    else if (cross0.z() < 0 && cross1.z() < 0 && cross2.z() < 0)return true;
    else return false;
}


static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}


void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind)
    {
        Triangle t;
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };
        //Homogeneous division 
        for (auto& vec : v) {
            vec /= vec.w();
        }
        //Viewport transformation  
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        super_rasterize_triangle(t);
    }
}


void rst::rasterizer::super_rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4();
    
    
    float minX, minY, maxX, maxY;
    minX = std::min(v[0].x(), std::min(v[1].x(), v[2].x()));
    maxX = std::max(v[0].x(), std::max(v[1].x(), v[2].x()));
    minY = std::min(v[0].y(), std::min(v[1].y(), v[2].y()));
    maxY = std::max(v[0].y(), std::max(v[1].y(), v[2].y()));

    for (int x = std::floor(minX); x <= std::ceil(maxX); x++) {
        for (int y = std::floor(minY); y <= std::ceil(maxY); y++) {
            int cnt = 0;
            for (int offset = 0; offset < 4; offset++) {
                if (insideTriangle(x + superOffsetX[offset], y + superOffsetY[offset],t.v)){
                    float alpha, beta, gamma;
                    std::tie(alpha, beta, gamma) = computeBarycentric2D(x + superOffsetX[offset], y + superOffsetY[offset], t.v);
                    float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                    float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                    z_interpolated *= w_reciprocal;
                
                    if (z_interpolated < super_depth_buf[get_super_index(x * 2 + offset % 2,y * 2 + offset / 2)]) {
                        cnt ++ ;
                        super_depth_buf[get_super_index(x * 2 + offset % 2,y * 2 + offset / 2)] = z_interpolated;
                        sample_color_buf[get_super_index(x * 2 + offset % 2,y * 2 + offset / 2)] = t.getColor();
                    }
                }
            
            }
            if(cnt > 0){
                Vector3f point = {(float)x, (float)y, 0};
                Vector3f color = {0, 0, 0};
                color += sample_color_buf[get_super_index(x * 2, y * 2)];
                color += sample_color_buf[get_super_index(x * 2 + 1, y * 2)];
                color += sample_color_buf[get_super_index(x * 2, y * 2 + 1)];
                color += sample_color_buf[get_super_index(x * 2 + 1, y * 2 + 1)];
                color /= 4.0f;
                set_pixel(point, color);
            }
        }
    }

}

//Screen space rasterization 
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4();
    
    
    float minX, minY, maxX, maxY;
    minX = std::min(v[0].x(), std::min(v[1].x(), v[2].x()));
    maxX = std::max(v[0].x(), std::max(v[1].x(), v[2].x()));
    minY = std::min(v[0].y(), std::min(v[1].y(), v[2].y()));
    maxY = std::max(v[0].y(), std::max(v[1].y(), v[2].y()));

    for (int x = std::floor(minX); x <= std::ceil(maxX); x++) {
        for (int y = std::floor(minY); y <= std::ceil(maxY); y++) {

            if (insideTriangle(x + 0.5, y + 0.5, t.v)) {
                float alpha, beta, gamma;
                std::tie(alpha, beta, gamma) = computeBarycentric2D(x, y, t.v);
                float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                z_interpolated *= w_reciprocal;

                int index = get_index(x, y);
                if (z_interpolated < depth_buf[index]) {
                    depth_buf[index] = z_interpolated;
                    auto color = t.getColor();
                    set_pixel(Eigen::Vector3f(x, y, z_interpolated), color);
                }
            }
            
            
        }
    }

    // TODO : Find out the bounding box of current triangle.
    // iterate through the pixel and find if the current pixel is inside the triangle


    // If so, use the following code to get the interpolated z value.
    //float alpha, beta, gamma;
    //std::tie(alpha, beta, gamma) = computeBarycentric2D(x, y, t.v);
    //float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    //float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
    //z_interpolated *= w_reciprocal;

    // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.

    

}


void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}


void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}


void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}


void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
        std::fill(sample_color_buf.begin(), sample_color_buf.end(), Eigen::Vector3f{0, 0, 0});
        
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
        std::fill(super_depth_buf.begin(), super_depth_buf.end(), std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
    super_depth_buf.resize(w * h * 4);
    sample_color_buf.resize(w * h * 4);
}


int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

int rst::rasterizer::get_super_index(int x, int y) {
    return (height * 2 - 1 - y) * width * 2 + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;

}



// clang-format on