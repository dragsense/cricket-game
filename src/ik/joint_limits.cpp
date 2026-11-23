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

bool parseBound(const std::string& s, LimitCoupling::Bound& out) {
    using B = LimitCoupling::Bound;
    static const std::pair<const char*, B> kNames[] = {{"twistMin", B::TwistMin},   {"twistMax", B::TwistMax},
                                                        {"swing1Min", B::Swing1Min}, {"swing1Max", B::Swing1Max},
                                                        {"swing2Min", B::Swing2Min}, {"swing2Max", B::Swing2Max}};
    for (const auto& [name, b] : kNames)
        if (s == name) {
            out = b;
            return true;
        }
    return false;
}

float& boundOf(JointLimit& l, LimitCoupling::Bound b) {
    using B = LimitCoupling::Bound;
    switch (b) {
    case B::TwistMin: return l.twistMin;
    case B::TwistMax: return l.twistMax;
    case B::Swing1Min: return l.swing1Min;
    case B::Swing1Max: return l.swing1Max;
    case B::Swing2Min: return l.swing2Min;
    default: return l.swing2Max;
    }
}

// The limit's own axis that a bound is on (0 = x, 1 = y, 2 = z).
int boundAxis(const JointLimit& l, LimitCoupling::Bound b) {
    using B = LimitCoupling::Bound;
    if (b == B::TwistMin || b == B::TwistMax) return l.axis;
    if (b == B::Swing1Min || b == B::Swing1Max) return swingAxis1(l.axis);
    return swingAxis2(l.axis);
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

float hingeAngle(math::Quat local, const JointLimit& hinge) {
    math::Quat q = math::normalize(local);
    if (q.w < 0.0f) q = -q;
    return twistAngle(q, unitAxis(hinge.axis));
}

JointLimit coupledLimit(const JointLimit& limit, float angle) {
    const LimitCoupling& c = limit.coupling;
    if (c.joint < 0 || c.angle1 == c.angle0) return limit;
    const float t = std::clamp((angle - c.angle0) / (c.angle1 - c.angle0), 0.0f, 1.0f);
    JointLimit out = limit;
    boundOf(out, c.bound) = c.value0 + (c.value1 - c.value0) * t;
    return out;
}

LimitFrame limitFrame(const JointLimit& lim) {
    LimitFrame f;
    if (lim.type == LimitType::Free) {
        f.min = math::Vec3(-math::kPi);
        f.max = math::Vec3(math::kPi);
        return f;
    }
    f.x = unitAxis(lim.axis);
    if (lim.type == LimitType::Hinge) {
        f.y = unitAxis((lim.axis + 1) % 3);
        f.z = math::cross(f.x, f.y);
        f.min = math::Vec3(lim.twistMin, 0.0f, 0.0f);
        f.max = math::Vec3(lim.twistMax, 0.0f, 0.0f);
    } else {
        f.y = unitAxis(swingAxis1(lim.axis));
        f.z = math::cross(f.x, f.y);
        const bool flip = math::dot(f.z, unitAxis(swingAxis2(lim.axis))) < 0.0f;
        f.min = math::Vec3(lim.twistMin, lim.swing1Min, flip ? -lim.swing2Max : lim.swing2Min);
        f.max = math::Vec3(lim.twistMax, lim.swing1Max, flip ? -lim.swing2Min : lim.swing2Max);
    }
    for (int a = 0; a < 3; ++a) {
        f.min[a] = std::clamp(f.min[a], -math::kPi, math::kPi);
        f.max[a] = std::clamp(f.max[a], -math::kPi, math::kPi);
    }
    return f;
}

namespace {

// One axis's range as Jolt's SwingTwistConstraintPart::SetLimits keeps it:
// locked within half a degree of 0, free past 179.5 deg each way, else the
// sine and cosine of its half angles.
struct AxisRange {
    bool locked = false;
    float halfMin = 0.0f, halfMax = 0.0f;  // half angles
    float sinMin = 0.0f, sinMax = 0.0f, cosMin = 1.0f, cosMax = 1.0f;
};

AxisRange axisRange(float lo, float hi) {
    const float kLocked = math::radians(0.5f), kFree = math::radians(179.5f);
    AxisRange r;
    r.halfMin = 0.5f * lo;
    r.halfMax = 0.5f * hi;
    if (lo > -kLocked && hi < kLocked) {
        r.locked = true;
    } else if (lo < -kFree && hi > kFree) {
        r.sinMin = -1.0f;
        r.sinMax = 1.0f;
        r.cosMin = r.cosMax = 0.0f;
    } else {
        r.sinMin = std::sin(r.halfMin);
        r.sinMax = std::sin(r.halfMax);
        r.cosMin = std::cos(r.halfMin);
        r.cosMax = std::cos(r.halfMax);
    }
    return r;
}

// Jolt's sDistanceToMinShorter: outside the range, the nearer side by the
// sine of the half angle (a full turn apart: the other way round).
bool minIsNearer(float dMin, float dMax) {
    dMin = std::abs(dMin);
    if (dMin > 1.0f) dMin = 2.0f - dMin;
    dMax = std::abs(dMax);
    if (dMax > 1.0f) dMax = 2.0f - dMax;
    return dMin < dMax;
}

// A one-axis part (the twist's x, or a swing's y or z with the other swing
// axis locked) held to its range; true if it moved. `c` is the part's w.
bool clampOne(const AxisRange& r, float& s, float& c) {
    const float dMin = r.sinMin - s, dMax = s - r.sinMax;
    if (dMin <= 0.0f && dMax <= 0.0f) return false;
    if (minIsNearer(dMin, dMax)) {
        s = r.sinMin;
        c = r.cosMin;
    } else {
        s = r.sinMax;
        c = r.cosMax;
    }
    return true;
}

} // namespace

namespace {

// q = swing * twist in the limit's frame, twist about X (Jolt's
// Quat::GetSwingTwist), both parts with w >= 0 (Jolt's ClampSwingTwist).
struct SwingTwist {
    float tx = 0.0f, tw = 1.0f, sy = 0.0f, sz = 0.0f, sw = 1.0f;
    bool negSwing = false, negTwist = false;
};

SwingTwist swingTwist(math::Quat q, const LimitFrame& f) {
    const math::Vec3 v(q.x, q.y, q.z);
    const float w = q.w, x = math::dot(v, f.x), y = math::dot(v, f.y), z = math::dot(v, f.z);
    SwingTwist p;
    p.sy = y;
    p.sz = z;
    p.sw = w;
    const float s = std::sqrt(w * w + x * x);
    if (s != 0.0f) {
        p.tx = x / s;
        p.tw = w / s;
        p.sy = (w * y - x * z) / s;
        p.sz = (w * z + x * y) / s;
        p.sw = s;
    }
    p.negSwing = p.sw < 0.0f;
    p.negTwist = p.tw < 0.0f;
    if (p.negSwing) {
        p.sy = -p.sy;
        p.sz = -p.sz;
        p.sw = -p.sw;
    }
    if (p.negTwist) {
        p.tx = -p.tx;
        p.tw = -p.tw;
    }
    return p;
}

} // namespace

math::Vec3 limitAngles(math::Quat q, const JointLimit& lim) {
    if (lim.type == LimitType::Free) return math::Vec3(0.0f);
    const SwingTwist p = swingTwist(math::normalize(q), limitFrame(lim));
    return math::Vec3(2.0f * std::atan2(p.tx, p.tw), 2.0f * std::atan2(p.sy, p.sw), 2.0f * std::atan2(p.sz, p.sw));
}

math::Quat clampRotation(math::Quat q, const JointLimit& lim) {
    if (lim.type == LimitType::Free) return q;
    q = math::normalize(q);
    const LimitFrame f = limitFrame(lim);
    SwingTwist parts = swingTwist(q, f);
    float &tx = parts.tx, &tw = parts.tw, &sy = parts.sy, &sz = parts.sz, &sw = parts.sw;
    const bool negSwing = parts.negSwing, negTwist = parts.negTwist;
    const AxisRange rx = axisRange(f.min.x, f.max.x), ry = axisRange(f.min.y, f.max.y), rz = axisRange(f.min.z, f.max.z);
    bool clamped = false;
    if (rx.locked) {
        clamped = tx != 0.0f;
        tx = 0.0f;
        tw = 1.0f;
    } else {
        clamped = clampOne(rx, tx, tw);
    }
    if (ry.locked && rz.locked) {
        clamped = clamped || sy != 0.0f || sz != 0.0f;
        sy = sz = 0.0f;
        sw = 1.0f;
    } else if (ry.locked || rz.locked) {
        // One swing axis: the locked one's part goes, the other keeps its
        // range (a clamped part is set to its bound, else kept as it is).
        float& off = ry.locked ? sy : sz;
        float& on = ry.locked ? sz : sy;
        const bool offClamped = off != 0.0f;
        float c = sw;
        if (clampOne(ry.locked ? rz : ry, on, c)) {
            off = 0.0f;
            sw = c;
            clamped = true;
        } else if (offClamped) {
            off = 0.0f;
            sw = std::sqrt(std::max(0.0f, 1.0f - on * on));
            clamped = true;
        }
    } else {
        // Pyramid: the half angle about Y and about Z, each to its range,
        // the swing rebuilt from them without a twist part.
        const float hy = std::atan2(sy, sw), hz = std::atan2(sz, sw);
        const float cy = std::clamp(hy, ry.halfMin, ry.halfMax), cz = std::clamp(hz, rz.halfMin, rz.halfMax);
        if (cy != hy || cz != hz) {
            sy = std::sin(cy) * std::cos(cz);
            sz = std::cos(cy) * std::sin(cz);
            sw = std::cos(cy) * std::cos(cz);
            const float n = std::sqrt(sy * sy + sz * sz + sw * sw);
            sy /= n;
            sz /= n;
            sw /= n;
            clamped = true;
        }
    }
    if (!clamped) return q;
    if (negSwing) {
        sy = -sy;
        sz = -sz;
        sw = -sw;
    }
    if (negTwist) {
        tx = -tx;
        tw = -tw;
    }
    // swing * twist, back from the joint frame.
    const float rw = sw * tw, qx = sw * tx, qy = sy * tw + sz * tx, qz = sz * tw - sy * tx;
    const math::Vec3 r = qx * f.x + qy * f.y + qz * f.z;
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
    // A coupled bound on a flipped axis: min and max swap, the values negate
    // (the coupled hinge is resolved per side by the caller).
    LimitCoupling& c = m.coupling;
    if (c.joint >= 0 && boundAxis(lim, c.bound) != 0) {
        using B = LimitCoupling::Bound;
        const B swapped[] = {B::TwistMax, B::TwistMin, B::Swing1Max, B::Swing1Min, B::Swing2Max, B::Swing2Min};
        c.bound = swapped[static_cast<int>(c.bound)];
        c.value0 = -c.value0;
        c.value1 = -c.value1;
    }
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
        // Optional "coupledTo": {"bone", "bound", "angleDeg": [a0, a1],
        // "valueDeg": [v0, v1]}: one bound set by another hinge's angle,
        // resolved per side (foot_l's by shin_l).
        int coupledPlain = -1, coupledLeft = -1, coupledRight = -1;
        if (auto cj = j.find("coupledTo"); cj != j.end() && cj->is_object()) {
            const std::string bone = cj->value("bone", "");
            const auto a = cj->find("angleDeg"), v = cj->find("valueDeg");
            if (!parseBound(cj->value("bound", ""), lim.coupling.bound) || a == cj->end() || v == cj->end() ||
                !a->is_array() || !v->is_array() || a->size() != 2 || v->size() != 2) {
                LOG_ERROR("joint limits: invalid coupledTo on '{}'", name);
                return false;
            }
            coupledPlain = rig.findJoint(bone);
            coupledLeft = rig.findJoint(bone + "_l");
            coupledRight = rig.findJoint(bone + "_r");
            lim.coupling.angle0 = math::radians((*a)[0].get<float>());
            lim.coupling.angle1 = math::radians((*a)[1].get<float>());
            lim.coupling.value0 = math::radians((*v)[0].get<float>());
            lim.coupling.value1 = math::radians((*v)[1].get<float>());
            lim.coupling.joint = 0;  // set per side below
            boundOf(lim, lim.coupling.bound) = lim.coupling.value0;
        }
        auto coupled = [](JointLimit l, int joint) {
            if (l.coupling.joint >= 0) l.coupling.joint = joint;
            return l;
        };
        set(plain, coupled(lim, coupledPlain));
        set(left, coupled(lim, coupledLeft));
        set(right, coupled(mirrored(lim), coupledRight));
    }
    // A coupled joint must be a hinge about X (its angle, the same sign on
    // both sides, is the coupling's input).
    for (const JointLimit& l : limits_) {
        const int cj = l.coupling.joint;
        if (cj < 0) continue;
        const JointLimit& h = limits_[static_cast<size_t>(cj)];
        if (h.type != LimitType::Hinge || h.axis != 0) {
            LOG_ERROR("joint limits: a coupled joint must be a hinge about x");
            return false;
        }
    }
    LOG_INFO("joint limits: {} joints limited", count);
    return true;
}

JointLimit JointLimits::limitIn(int joint, const rig::Pose& pose) const {
    const JointLimit& l = limits_[static_cast<size_t>(joint)];
    const int cj = l.coupling.joint;
    if (cj < 0 || static_cast<size_t>(cj) >= pose.locals.size()) return l;
    return coupledLimit(l, hingeAngle(pose.locals[static_cast<size_t>(cj)].rotation, limits_[static_cast<size_t>(cj)]));
}

void JointLimits::apply(rig::Pose& pose) const {
    // Parents come first in the rig, so a coupled joint (the knee) is
    // clamped before the joint it bounds (the ankle).
    for (size_t i = 0; i < limits_.size() && i < pose.locals.size(); ++i)
        pose.locals[i].rotation = clampRotation(pose.locals[i].rotation, limitIn(static_cast<int>(i), pose));
}

} // namespace ik
