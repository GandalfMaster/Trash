//
// Created by LEI XU on 4/11/19.
//

#include "Triangle.hpp"
#include <algorithm>
#include <array>

//构造函数，初始化三角形顶点，颜色，纹理坐标
Triangle::Triangle() {
    v[0] << 0,0,0;
    v[1] << 0,0,0;
    v[2] << 0,0,0;

    color[0] << 0.0, 0.0, 0.0;
    color[1] << 0.0, 0.0, 0.0;
    color[2] << 0.0, 0.0, 0.0;

    tex_coords[0] << 0.0, 0.0;
    tex_coords[1] << 0.0, 0.0;
    tex_coords[2] << 0.0, 0.0;
}

//设置顶点
void Triangle::setVertex(int ind, Vector3f ver){
    v[ind] = ver;
}

//设置法线
void Triangle::setNormal(int ind, Vector3f n){
    normal[ind] = n;
}

//设置颜色
void Triangle::setColor(int ind, float r, float g, float b) {
    if((r<0.0) || (r>255.) ||
       (g<0.0) || (g>255.) ||
       (b<0.0) || (b>255.)) {
        fprintf(stderr, "ERROR! Invalid color values");
        fflush(stderr);
        exit(-1);
    }

    color[ind] = Vector3f((float)r/255.,(float)g/255.,(float)b/255.);
    return;
}

//设置纹理坐标
void Triangle::setTexCoord(int ind, float s, float t) {
    tex_coords[ind] = Vector2f(s,t);
}

//将三角形顶点转换为4D向量
std::array<Vector4f, 3> Triangle::toVector4() const
{
    std::array<Eigen::Vector4f, 3> res;
    std::transform(std::begin(v), std::end(v), res.begin(), [](auto& vec) { return Eigen::Vector4f(vec.x(), vec.y(), vec.z(), 1.f); });
    return res;
}
