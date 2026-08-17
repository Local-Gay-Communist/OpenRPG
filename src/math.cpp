// math.cpp
#include "math.h"
#include <cmath>

Mat4::Mat4() {
    for(int i=0;i<16;i++) m[i]=0.0f;
}

Mat4 identity() {
    Mat4 r;
    r.m[0] = 1.0f; r.m[5] = 1.0f; r.m[10] = 1.0f; r.m[15] = 1.0f;
    return r;
}

Mat4 rotate_x(float angle) {
    Mat4 r = identity();
    float c = cosf(angle), s = sinf(angle);
    r.m[5] = c;   r.m[6] = s;
    r.m[9] = -s;  r.m[10] = c;
    return r;
}

Mat4 rotate_y(float angle) {
    Mat4 r = identity();
    float c = cosf(angle), s = sinf(angle);
    r.m[0] = c;   r.m[2] = -s;
    r.m[8] = s;   r.m[10] = c;
    return r;
}

Mat4 rotate_z(float angle) {
    Mat4 r = identity();
    float c = cosf(angle), s = sinf(angle);
    r.m[0] = c;   r.m[1] = s;
    r.m[4] = -s;  r.m[5] = c;
    return r;
}

Mat4 look_at(float ex, float ey, float ez, float cx, float cy, float cz, float ux, float uy, float uz) {
    float fx = cx - ex, fy = cy - ey, fz = cz - ez;
    float flen = sqrtf(fx*fx + fy*fy + fz*fz);
    if(flen < 0.0001f) return identity();
    fx /= flen; fy /= flen; fz /= flen;

    float rx = uy*fz - uz*fy;
    float ry = uz*fx - ux*fz;
    float rz = ux*fy - uy*fx;
    float rlen = sqrtf(rx*rx + ry*ry + rz*rz);
    if(rlen < 0.0001f) { rx = 1; ry = 0; rz = 0; }
    else { rx /= rlen; ry /= rlen; rz /= rlen; }

    float ux2 = fy*rz - fz*ry;
    float uy2 = fz*rx - fx*rz;
    float uz2 = fx*ry - fy*rx;

    Mat4 r;
    r.m[0] = rx;  r.m[4] = ry;  r.m[8] = rz;  r.m[12] = -(rx*ex + ry*ey + rz*ez);
    r.m[1] = ux2; r.m[5] = uy2; r.m[9] = uz2; r.m[13] = -(ux2*ex + uy2*ey + uz2*ez);
    r.m[2] = -fx; r.m[6] = -fy; r.m[10] = -fz; r.m[14] = (fx*ex + fy*ey + fz*ez);
    r.m[3] = 0;   r.m[7] = 0;   r.m[11] = 0;   r.m[15] = 1;
    return r;
}

Mat4 perspective(float fov, float aspect, float near, float far) {
    Mat4 r{};
    float tanHalf = tanf(fov / 2.0f);
    r.m[0]  = 1.0f / (aspect * tanHalf);
    r.m[5]  = 1.0f / tanHalf;
    r.m[10] = -(far + near) / (far - near);
    r.m[11] = -1.0f;
    r.m[14] = -(2.0f * far * near) / (far - near);
    return r;
}

Mat4 mat4_mul(const Mat4& a, const Mat4& b) {
    Mat4 r;
    for (int col = 0; col < 4; col++) {
        for (int row = 0; row < 4; row++) {
            float sum = 0.0f;
            for (int k = 0; k < 4; k++) {
                sum += a.m[k*4 + row] * b.m[col*4 + k];
            }
            r.m[col*4 + row] = sum;
        }
    }
    return r;
}