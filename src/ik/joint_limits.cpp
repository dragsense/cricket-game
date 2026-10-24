#include "ik/joint_limits.h"

#include "core/config.h"
#include "core/log.h"

#include <algorithm>
#include <cmath>
#include <string>

namespace ik {

namespace {

math::Vec3 unitAxis(int a) {
    math::Vec3 v(0.0f);
    v[a] = 1.0f;
    return v;
}

// Signed rotation angle of a pure twist quaternion about `axis`, in [-pi, pi].
float twistAngle(math::Quat twist, math::Vec3 axis) {
    float s = math::dot(math::Vec3(twist.x, twist.y, twist.z), axis);
    float a = 2.0f * std::atan2(s, twist.w);
    if (a > math::kPi) a -= math::kTwoPi;
    if (a < -math::kPi) a += math::kTwoPi;
    return a;
}

// The two swing axes are the axes other than the twist axis, in x, y, z order.
int swingAxis1(int twistAxis) { return twistAxis == 0 ? 1 : 0; }
int swingAxis2(int twistAxis) { return twistAxis == 2 ? 1 : 2; }

// Ellipse radius on one axis for the side the value is on.
float radiusFor(float v, float minV, float maxV) { return v >= 0.0f ? maxV : -minV; }

int parseAxis(const std::string& s) { return s == "x" ? 0 : s == "z" ? 2 : 1; }

bool readRange(const nlohmann::json& j, const char* key, float& lo, float& hi) {
    auto it = j.find(key);
    if (it == j.end() || !it->is_array() || it->size() != 2) return false;
    lo = math::radians((*it)[0].get<float>());
    hi = math::radians((*it)[1].get<float>());
    return lo <= hi;
}

bool parseLimit(const nlohmann::json& j, JointLimit& out) {
    std::string type = j.value("type", "");
    if (type == "hinge") {
        out.type = LimitType::Hinge;
        out.axis = parseAxis(j.value("axis", "x"));
        out.twistMin = math::radians(j.value("min", 0.0f));
        out.twistMax = math::radians(j.value("max", 0.0f));
        return out.twistMin <= out.twistMax;
    }
    if (type == "swingTwist") {
        out.type = LimitType::SwingTwist;
        out.axis = parseAxis(j.value("twistAxis", "y"));
        return readRange(j, "twist", out.twistMin, out.twistMax) &&
               readRange(j, "swing1", out.swing1Min, out.swing1Max) &&
               readRange(j, "swing2", out.swing2Min, out.swing2Max) && out.swing1Min <= 0.0f &&
               out.swing1Max >= 0.0f && out.swing2Min <= 0.0f && out.swing2Max >= 0.0f;
    }
    return false;
}

} // namespace

math::Quat clampRotation(math::Quat q, const JointLimit& lim) {
    if (lim.type == LimitType::Free) return q;
    q = math::normalize(q);
    if (q.w < 0.0f) q = -q;

    const math::Vec3 axis = unitAxis(lim.axis);
    math::Quat swing, twist;
    math::swingTwist(q, axis, swing, twist);

    float t = std::clamp(twistAngle(twist, axis), lim.twistMin, lim.twistMax);
    math::Quat clampedTwist = math::fromAxisAngle(axis, t);
    if (lim.type == LimitType::Hinge) return clampedTwist;

    // Swing as a rotation vector in the plane perpendicular to the twist axis.
    if (swing.w < 0.0f) swing = -swing;
    math::Vec3 sv(swing.x, swing.y, swing.z);
    float sinHalf = math::length(sv);
    if (sinHalf < 1e-7f) return clampedTwist;
    float angle = 2.0f * std::atan2(sinHalf, swing.w);
    math::Vec3 rv = sv * (angle / sinHalf);

    const math::Vec3 ax1 = unitAxis(swingAxis1(lim.axis));
    const math::Vec3 ax2 = unitAxis(swingAxis2(lim.axis));
    float a = math::dot(rv, ax1);
    float b = math::dot(rv, ax2);
    float ra = radiusFor(a, lim.swing1Min, lim.swing1Max);
    float rb = radiusFor(b, lim.swing2Min, lim.swing2Max);

    // Zero-width directions lock that component; the rest is an ellipse.
    if (ra <= 0.0f) a = 0.0f;
    if (rb <= 0.0f) b = 0.0f;
    float e = (ra > 0.0f ? (a / ra) * (a / ra) : 0.0f) + (rb > 0.0f ? (b / rb) * (b / rb) : 0.0f);
    if (e > 1.0f) {
        float s = 1.0f / std::sqrt(e);
        a *= s;
        b *= s;
    }
    math::Vec3 clamped = a * ax1 + b * ax2;
    float len = math::length(clamped);
    math::Quat clampedSwing = len > 1e-7f ? math::fromAxisAngle(clamped / len, len) : math::identityQuat();
    return math::normalize(clampedSwing * clampedTwist);
}

JointLimit mirrored(const JointLimit& lim) {
    // Reflecting across the YZ plane maps a rotation vector (x, y, z) to (x, -y, -z).
    auto flip = [](float& lo, float& hi) {
        float l = lo;
        lo = -hi;
        hi = -l;
    };
    JointLimit m = lim;
    if (lim.axis != 0) flip(m.twistMin, m.twistMax);
    if (swingAxis1(lim.axis) != 0) flip(m.swing1Min, m.swing1Max);
    if (swingAxis2(lim.axis) != 0) flip(m.swing2Min, m.swing2Max);
    return m;
}

bool JointLimits::load(const core::Config& config, const rig::Rig& rig) {
    limits_.assign(static_cast<size_t>(rig.numJoints()), JointLimit{});
    auto it = config.root().find("joints");
    if (it == config.root().end() || !it->is_object()) {
        LOG_ERROR("joint limits: no \"joints\" block");
        return false;
    }
    int count = 0;
    for (const auto& [name, j] : it->items()) {
        JointLimit lim;
        if (!parseLimit(j, lim)) {
            LOG_ERROR("joint limits: invalid entry '{}'", name);
            return false;
        }
        int plain = rig.findJoint(name);
        int left = rig.findJoint(name + "_l");
        int right = rig.findJoint(name + "_r");
        if (plain < 0 && left < 0) {
            LOG_ERROR("joint limits: unknown joint '{}'", name);
            return false;
        }
        auto set = [&](int joint, const JointLimit& l) {
            if (joint < 0) return;
            limits_[static_cast<size_t>(joint)] = l;
            ++count;
        };
        set(plain, lim);
        set(left, lim);
        set(right, mirrored(lim));
    }
    LOG_INFO("joint limits: {} joints limited", count);
    return true;
}

void JointLimits::apply(rig::Pose& pose) const {
    for (size_t i = 0; i < limits_.size() && i < pose.locals.size(); ++i)
        pose.locals[i].rotation = clampRotation(pose.locals[i].rotation, limits_[i]);
}

} // namespace ik
