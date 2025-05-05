// ============================================================================
//  advanced_bsdf.cpp  – full, working implementation
// ============================================================================
#include "bsdf.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <utility>

#include "application/visual_debugger.h"

using std::max;
using std::min;
using std::swap;

namespace CGL {

// -------------------------------------------------------------
//  Helpers
// -------------------------------------------------------------
static inline Vector3D schlick(const Vector3D &F0, double cos_i) {
    return F0 + (Vector3D(1.0) - F0) * pow(1.0 - cos_i, 5.0);
}

static inline double schlick_scalar(double cos_i, double eta_i, double eta_t) {
    double r0 = (eta_i - eta_t) / (eta_i + eta_t);
    r0 *= r0;
    return r0 + (1.0 - r0) * pow(1.0 - cos_i, 5.0);
}

// -------------------------------------------------------------
//  Mirror BSDF
// -------------------------------------------------------------
Vector3D MirrorBSDF::f(const Vector3D /*wo*/, const Vector3D /*wi*/) {
    return Vector3D(); // delta ⇒ zero everywhere in the PDF domain
}

Vector3D MirrorBSDF::sample_f(const Vector3D wo, Vector3D *wi, double *pdf) {
    *wi = Vector3D(-wo.x, -wo.y, wo.z);  // perfect reflection
    wi->normalize();
    *pdf = 1.0;                          // delta
    return reflectance / fabs(wi->z);
}

void MirrorBSDF::render_debugger_node() {
    if (ImGui::TreeNode(this, "Mirror BSDF")) {
        DragDouble3("Reflectance", &reflectance[0], 0.005);
        ImGui::TreePop();
    }
}

// -------------------------------------------------------------
//  Micro-facet BSDF (Cook–Torrance, Beckmann)
// -------------------------------------------------------------
double MicrofacetBSDF::G(const Vector3D wo, const Vector3D wi) {
    return 1.0 / (1.0 + Lambda(wo) + Lambda(wi));
}

double MicrofacetBSDF::D(const Vector3D h) {
    if (h.z <= 0.0) return 0.0;
    double cos_h2 = h.z * h.z;
    double tan_h2 = (1.0 - cos_h2) / cos_h2;
    double a2     = alpha * alpha;
    return exp(-tan_h2 / a2) / (PI * a2 * cos_h2 * cos_h2);
}

Vector3D MicrofacetBSDF::F(const Vector3D wi, const Vector3D h) {
    double cos_i_h = fabs(dot(wi, h));
    Vector3D one(1.0);
    Vector3D F0 = ((eta - one) * (eta - one) + k * k) /
                  ((eta + one) * (eta + one) + k * k);
    return schlick(F0, cos_i_h);
}

Vector3D MicrofacetBSDF::f(const Vector3D wo, const Vector3D wi) {
    if (wo.z <= 0.0 || wi.z <= 0.0) return Vector3D();
    Vector3D h = (wo + wi).unit();
    return F(wi, h) * (D(h) * G(wo, wi) / (4.0 * wo.z * wi.z));
}

Vector3D MicrofacetBSDF::sample_f(const Vector3D wo, Vector3D *wi, double *pdf) {
    Vector2D u = sampler.get_sample();
    double phi = 2.0 * PI * u.y;
    double tan2_theta = -alpha * alpha * log(1.0 - u.x);
    double theta = atan(sqrt(tan2_theta));

    double cos_h = cos(theta);
    double sin_h = sin(theta);

    Vector3D h(sin_h * cos(phi), sin_h * sin(phi), cos_h);

    double dot_wo_h = dot(wo, h);
    *wi = -wo + 2.0 * dot_wo_h * h;

    if (wi->z <= 0.0) { 
        *pdf = 1.0; 
        return Vector3D(); 
    }

    double p_h = D(h) * cos_h;
    *pdf       = p_h / (4.0 * fabs(dot_wo_h));

    return f(wo, *wi);
}

void MicrofacetBSDF::render_debugger_node() {
    if (ImGui::TreeNode(this, "Microfacet BSDF")) {
        DragDouble3("eta", &eta[0], 0.005);
        DragDouble3("k",   &k[0],   0.005);
        DragDouble ("alpha", &alpha, 0.005);
        ImGui::TreePop();
    }
}

// -------------------------------------------------------------
//  Refraction BSDF (pure dielectric)
// -------------------------------------------------------------
Vector3D RefractionBSDF::f(const Vector3D /*wo*/, const Vector3D /*wi*/) { return Vector3D(); }

Vector3D RefractionBSDF::sample_f(const Vector3D wo, Vector3D *wi, double *pdf) {
    if (!refract(wo, wi, ior)) { *pdf = 0.0; return Vector3D(); }
    *pdf = 1.0; // delta
    return transmittance / (abs_cos_theta(*wi) * ior * ior);
}

void RefractionBSDF::render_debugger_node() {
    if (ImGui::TreeNode(this, "Refraction BSDF")) {
        DragDouble3("Transmittance", &transmittance[0], 0.005);
        DragDouble ("ior", &ior, 0.005);
        ImGui::TreePop();
    }
}

// -------------------------------------------------------------
//  Glass BSDF  (reflection + refraction, Fresnel-weighted)
// -------------------------------------------------------------
Vector3D GlassBSDF::f(const Vector3D /*wo*/, const Vector3D /*wi*/) { return Vector3D(); }

Vector3D GlassBSDF::sample_f(const Vector3D wo, Vector3D *wi, double *pdf) {
    double cos_i    = cos_theta(wo);
    bool   entering = cos_i > 0.0;

    double eta_i = entering ? 1.0 : ior;
    double eta_t = entering ? ior : 1.0;

    double fr = schlick_scalar(fabs(cos_i), eta_i, eta_t);

    if (coin_flip(fr)) {
        // —— reflection branch ——
        reflect(wo, wi);
        *pdf = fr;
        return 0.8 * reflectance / abs_cos_theta(*wi);
    } else {
        // —— refraction branch ——
        if (!refract(wo, wi, ior)) {   // total internal reflection fallback
            reflect(wo, wi);
            *pdf = 1.0;
            return reflectance / abs_cos_theta(*wi);
        }
        double eta = eta_i / eta_t;
        *pdf = 1.0 - fr;
        return 0.8 *transmittance / (abs_cos_theta(*wi) * eta * eta);
    }
}

void GlassBSDF::render_debugger_node() {
    if (ImGui::TreeNode(this, "Glass BSDF")) {
        DragDouble3("Reflectance",   &reflectance[0],   0.005);
        DragDouble3("Transmittance", &transmittance[0], 0.005);
        DragDouble ("ior",           &ior,             0.005);
        ImGui::TreePop();
    }
}

// -------------------------------------------------------------
//  Shared utility functions
// -------------------------------------------------------------
void BSDF::reflect(const Vector3D wo, Vector3D *wi) {
    *wi = Vector3D(-wo.x, -wo.y, wo.z);
    wi->normalize();
}

bool BSDF::refract(const Vector3D wo, Vector3D *wi, double ior) {
    double cos_i    = cos_theta(wo);
    bool   entering = cos_i > 0.0;

    double eta_i = entering ? 1.0 : ior;
    double eta_t = entering ? ior : 1.0;
    double eta   = eta_i / eta_t;

    double sin2_t = eta * eta * max(0.0, 1.0 - cos_i * cos_i);
    if (sin2_t >= 1.0) return false; // TIR

    double cos_t = sqrt(1.0 - sin2_t);
    if (entering) cos_t = -cos_t;

    *wi = Vector3D(-eta * wo.x, -eta * wo.y, cos_t);
    wi->normalize();
    return true;
}

} // namespace CGL
