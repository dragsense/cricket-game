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

    // Swing-twist: measured and clamped the way Jolt's SixDOF joint does
    // (pyramid swing; the physics body uses the same ranges, so both read
    // them alike). Joint frame: X = the twist axis, Y = the first swing axis,
    // Z = X x Y (the second swing axis, or its opposite: then its range flips).
    const math::Vec3 fx = axis, fy = unitAxis(swingAxis1(lim.axis)), fz = math::cross(fx, fy);
    const bool flip = math::dot(fz, unitAxis(swingAxis2(lim.axis))) < 0.0f;
    const float zMin = flip ? -lim.swing2Max : lim.swing2Min, zMax = flip ? -lim.swing2Min : lim.swing2Max;
    const math::Vec3 v(q.x, q.y, q.z);
    const float w = q.w, x = math::dot(v, fx), y = math::dot(v, fy), z = math::dot(v, fz);

    // q = swing * twist, twist about X (Jolt's Quat::GetSwingTwist).
    const float s = std::sqrt(w * w + x * x);
    float tx = 0.0f, tw = 1.0f, sy = y, sz = z, sw = 0.0f;
    if (s > 1e-7f) {
        tx = x / s;
        tw = w / s;
        sy = (w * y - x * z) / s;
        sz = (w * z + x * y) / s;
        sw = s;
    }
    // Twist: its angle clamped to its range.
    const float twistHalf = std::clamp(std::atan2(tx, tw), 0.5f * lim.twistMin, 0.5f * lim.twistMax);
    tx = std::sin(twistHalf);
    tw = std::cos(twistHalf);
    // Swing (pyramid): the angle about Y then Z, each clamped to its range,
    // and the swing rebuilt from them (Jolt's ClampSwingTwist).
    const float hy = std::atan2(sy, sw), hz = std::atan2(sz, sw);
    const float cy = std::clamp(hy, 0.5f * lim.swing1Min, 0.5f * lim.swing1Max);
    const float cz = std::clamp(hz, 0.5f * zMin, 0.5f * zMax);
    if (cy != hy || cz != hz) {
        sy = std::sin(cy) * std::cos(cz);
        sz = std::cos(cy) * std::sin(cz);
        sw = std::cos(cy) * std::cos(cz);
        const float n = std::sqrt(sy * sy + sz * sz + sw * sw);
        sy /= n;
        sz /= n;
        sw /= n;
    }
    // swing * twist, back from the joint frame.
    const float rw = sw * tw, rx = sw * tx, ry = sy * tw + sz * tx, rz = sz * tw - sy * tx;
    const math::Vec3 r = rx * fx + ry * fy + rz * fz;
    return math::normalize(math::Quat(rw, r.x, r.y, r.z));
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
