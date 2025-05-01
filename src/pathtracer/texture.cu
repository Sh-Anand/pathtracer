#include "texture.h"

namespace CGL {
    DEVICE Vector4D CudaTexture::sample(const Vector2D &uv) {
        // wrap or clamp your UVs as needed
        float u_f = uv.x - floorf(uv.x);
        float v_f = uv.y - floorf(uv.y);
        // // flip v if your image origin is top-left:
        v_f = 1.0f - v_f;

        int u = int(u_f * (width  - 1) + 0.5f);
        int v = int(v_f * (height - 1) + 0.5f);

        // clamp to valid
        u = max(0, min(u, width  - 1));
        v = max(0, min(v, height - 1));

        // compute byte index
        int comps = has_alpha ? 4 : 3;
        size_t idx = (size_t(v) * width + size_t(u)) * comps;
        const uint8_t *base = data;

        uchar4 c;
        if (has_alpha) {
        // RGBA8
        c.x = base[idx + 0];
        c.y = base[idx + 1];
        c.z = base[idx + 2];
        c.w = base[idx + 3];
        } else {
        // RGB8 → treat alpha = 255
        c.x = base[idx + 0];
        c.y = base[idx + 1];
        c.z = base[idx + 2];
        c.w = 255;
        }
        return Vector4D(c.x, c.y, c.z, c.w) * RGB_R;
    }

    DEVICE Vector3D CudaTexture::perturb(const Vector2D &uv, const Vector4D &tangent, const Vector3D &n) {
        Vector4D c = sample(uv);
        Vector3D n_tangent = Vector3D (c.x, c.y, c.z) * 2.0f - Vector3D(1.0f);

        Vector3D T = Vector3D(tangent.x, tangent.y, tangent.z).unit();
        Vector3D B = tangent.w * cross(n, T).unit();

        Vector3D perturbed =
                    (T * n_tangent.x +
                     B * n_tangent.y +
                     n * n_tangent.z).unit();

        printf ("normal: %f %f %f, perturbed: %f %f %f\n", n.x, n.y, n.z, perturbed.x, perturbed.y, perturbed.z);

        return perturbed;
    }
}