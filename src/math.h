#pragma once
struct Mat4 {
    float m[16];
    Mat4();
};
Mat4 identity();
Mat4 rotate_x(float angle);
Mat4 rotate_y(float angle);
Mat4 rotate_z(float angle);
Mat4 look_at(float ex, float ey, float ez, float cx, float cy, float cz, float ux, float uy, float uz);
Mat4 perspective(float fov, float aspect, float near, float far);
Mat4 mat4_mul(const Mat4& a, const Mat4& b);