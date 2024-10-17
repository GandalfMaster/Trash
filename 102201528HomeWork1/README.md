<div style="text-align: center; font-size: 36px">
计算机图形学
</div>

<div style="text-align: center;">
第一次作业：光栅化三角形
</div>

## 一.前置函数实现

实现一个函数，来判断点是否在一个三角形的内部.

```c++
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
```

这个代码通过叉乘来计算出点是否在三角形的内部，主要通过三个叉乘结果的Z值正负是否相同来判断



## 二.实现三角形光栅化

接下来我们可以实现光栅化函数。

```C++
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

}
```

这个代码的实现结果如下，可以看到图像边沿有明显的锯齿感

![](photo\1.png)

这是因为我们的采样不够细，可以进一步采用超采样来实现

## 三.实现超采样光栅化

现在实现了一个超采样的光栅化函数

```C++
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
```

这个函数把原来的一个单元网格进一步细分，分为四个网格来进行渲染，进一步削弱锯齿感

通过这个函数，我们可以看到基本实现了平滑的三角形。

![2](photo\2.png)
