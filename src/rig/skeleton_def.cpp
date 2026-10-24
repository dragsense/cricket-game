#include "rig/skeleton_def.h"

#include "core/config.h"
#include "core/log.h"

#include <cmath>
#include <limits>

namespace rig {

namespace {

math::Vec3 readVec3(const nlohmann::json& j, const char* key, math::Vec3 fallback) {
    auto it = j.find(key);
    if (it == j.end() || !it->is_array() || it->size() != 3) return fallback;
    return {(*it)[0].get<float>(), (*it)[1].get<float>(), (*it)[2].get<float>()};
}

math::Vec3 mirrorX(math::Vec3 v) { return {-v.x, v.y, v.z}; }

bool readVisual(const nlohmann::json& j, BoneVisual& v) {
    auto it = j.find("visual");
    if (it == j.end()) return true;
    std::string shape = it->value("shape", "");
    if (shape == "capsule") {
        v.shape = VisualShape::Capsule;
    } else if (shape == "box") {
        v.shape = VisualShape::Box;
    } else {
        LOG_ERROR("skeleton: unknown visual shape '{}'", shape);
        return false;
    }
    v.material = it->value("material", "");
    v.from = readVec3(*it, "from", v.from);
    v.to = readVec3(*it, "to", math::Vec3(std::numeric_limits<float>::quiet_NaN()));
    v.radius = it->value("radius", 0.0f);
    v.center = readVec3(*it, "center", v.center);
    v.size = readVec3(*it, "size", v.size);
    return true;
}

void mirrorBone(BoneDef& b) {
    b.offset = mirrorX(b.offset);
    b.tip = mirrorX(b.tip);
    b.visual.from = mirrorX(b.visual.from);
    if (!std::isnan(b.visual.to.x)) b.visual.to = mirrorX(b.visual.to);
    b.visual.center = mirrorX(b.visual.center);
}

} // namespace

int SkeletonDef::find(const std::string& name) const {
    for (size_t i = 0; i < bones.size(); ++i)
        if (bones[i].name == name) return static_cast<int>(i);
    return kNoParent;
}

float SkeletonDef::legLengthRatio() const {
    int shin = find("shin_l");
    int foot = find("foot_l");
    if (shin < 0 || foot < 0) return 0.0f;
    return length(bones[static_cast<size_t>(shin)].offset) + length(bones[static_cast<size_t>(foot)].offset);
}

bool loadSkeletonDef(const core::Config& config, SkeletonDef& out) {
    out = SkeletonDef{};
    const nlohmann::json& root = config.root();
    out.height = config.getFloat("height", out.height);
    if (auto it = root.find("heightRange"); it != root.end() && it->is_array() && it->size() == 2) {
        out.minHeight = (*it)[0].get<float>();
        out.maxHeight = (*it)[1].get<float>();
    }
    out.spawnPosition = config.getVec3("spawn.position", out.spawnPosition);
    out.spawnYaw = math::radians(config.getFloat("spawn.yawDeg", 0.0f));

    auto bonesIt = root.find("bones");
    if (bonesIt == root.end() || !bonesIt->is_array() || bonesIt->empty()) {
        LOG_ERROR("skeleton: no bones");
        return false;
    }

    for (const auto& j : *bonesIt) {
        BoneDef b;
        std::string name = j.value("name", "");
        std::string parent = j.value("parent", "");
        bool mirror = j.value("mirror", false);
        b.offset = readVec3(j, "offset", b.offset);
        b.tip = readVec3(j, "tip", b.tip);
        b.massPercent = j.value("mass", 0.0f);
        if (name.empty() || !readVisual(j, b.visual)) {
            LOG_ERROR("skeleton: bad bone entry '{}'", name);
            return false;
        }

        for (int side = 0; side < (mirror ? 2 : 1); ++side) {
            BoneDef s = b;
            const char* suffix = !mirror ? "" : (side == 0 ? "_l" : "_r");
            s.name = name + suffix;
            if (side == 1) mirrorBone(s);
            if (!parent.empty()) {
                // A mirrored parent carries the same side suffix.
                int p = out.find(parent + suffix);
                if (p < 0) p = out.find(parent);
                if (p < 0) {
                    LOG_ERROR("skeleton: bone '{}' has unknown parent '{}'", s.name, parent);
                    return false;
                }
                s.parent = p;
            } else if (!out.bones.empty()) {
                LOG_ERROR("skeleton: only the first bone may be the root ('{}')", s.name);
                return false;
            }
            out.bones.push_back(std::move(s));
        }
    }

    // Capsules without an explicit end run to the bone's first child (or tip).
    for (size_t i = 0; i < out.bones.size(); ++i) {
        BoneVisual& v = out.bones[i].visual;
        if (v.shape != VisualShape::Capsule || !std::isnan(v.to.x)) continue;
        v.to = out.bones[i].tip;
        for (size_t c = i + 1; c < out.bones.size(); ++c) {
            if (out.bones[c].parent == static_cast<int>(i)) {
                v.to = out.bones[c].offset;
                break;
            }
        }
    }

    LOG_INFO("skeleton: {} bones, height {:.2f} m, leg {:.3f} H", out.bones.size(), out.height, out.legLengthRatio());
    return true;
}

} // namespace rig
