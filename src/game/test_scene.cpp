#include "game/test_scene.h"

namespace game {

using math::Vec3;
using render::MaterialDesc;

namespace {

MaterialDesc readMaterial(const core::Config& c, const std::string& key, MaterialDesc m) {
    m.baseColor = c.getVec3(key + ".color", m.baseColor);
    m.roughness = c.getFloat(key + ".roughness", m.roughness);
    m.metallic = c.getFloat(key + ".metallic", m.metallic);
    m.stripeWidth = c.getFloat(key + ".stripeWidth", m.stripeWidth);
    m.stripeStrength = c.getFloat(key + ".stripeStrength", m.stripeStrength);
    m.seamColor = c.getVec3(key + ".seamColor", m.seamColor);
    m.clearCoat = c.getFloat(key + ".clearCoat", m.clearCoat);
    return m;
}

math::Transform at(Vec3 pos, math::Quat rot = math::identityQuat()) {
    math::Transform t;
    t.position = pos;
    t.rotation = rot;
    return t;
}

// Painted-line heights above the grass/pitch, to avoid z-fighting.
constexpr float kPitchY = 0.004f;
constexpr float kPaintY = 0.006f;

// The 30-yard fielding circle: two semicircles centred on each middle stump,
// joined by straight lines parallel to the pitch. Returns marker positions
// spaced roughly `spacing` metres apart along that outline.
std::vector<Vec3> fieldingCirclePoints(float radius, float halfPitch, float spacing) {
    std::vector<Vec3> pts;
    int arcCount = std::max(4, static_cast<int>(math::kPi * radius / spacing));
    int lineCount = std::max(1, static_cast<int>(2.0f * halfPitch / spacing));
    for (int end = 0; end < 2; ++end) {
        float sign = end == 0 ? 1.0f : -1.0f;
        // Semicircle behind this end's stumps.
        for (int i = 0; i <= arcCount; ++i) {
            float a = math::kPi * static_cast<float>(i) / static_cast<float>(arcCount);
            pts.emplace_back(radius * std::cos(a), 0.0f, sign * (halfPitch + radius * std::sin(a)));
        }
        // Straight side on this end's x side.
        for (int i = 1; i < lineCount; ++i) {
            float z = -halfPitch + 2.0f * halfPitch * static_cast<float>(i) / static_cast<float>(lineCount);
            pts.emplace_back(sign * radius, 0.0f, z);
        }
    }
    return pts;
}

} // namespace

TestScene buildTestScene(World& w, render::Renderer& r, const core::Config& c) {
    TestScene scene;
    // Every scene entity goes through add(), so the scene can be rebuilt on config reload.
    auto add = [&](const render::MeshData& mesh, const MaterialDesc& mat, const math::Transform& t, bool shadows) {
        entt::entity e = w.spawn(r, mesh, mat, t, shadows);
        scene.entities.push_back(e);
        return e;
    };
    scene.pitchLength = c.getFloat("pitch.length", 20.12f);
    float halfPitch = 0.5f * scene.pitchLength;

    // --- Round ground: outfield to the boundary, run-off beyond it. ---
    float boundary = c.getFloat("field.boundaryRadius", 70.0f);
    float runOff = c.getFloat("field.runOffWidth", 6.0f);
    MaterialDesc grass = readMaterial(c, "field.grass", {{0.27f, 0.52f, 0.18f}, 0.9f, 0.0f, 5.0f, 0.12f});
    add(render::makeDisc(boundary, 192), grass, at(Vec3(0.0f)), false);

    MaterialDesc runOffMat = readMaterial(c, "field.runOff", {{0.22f, 0.44f, 0.16f}, 0.95f, 0.0f, 0.0f, 0.0f});
    add(render::makeRing(boundary, boundary + runOff, 192), runOffMat, at(Vec3(0.0f)), false);

    float surroundRadius = c.getFloat("field.surroundRadius", 300.0f);
    MaterialDesc surround = readMaterial(c, "field.surround", {{0.35f, 0.37f, 0.33f}, 0.95f, 0.0f, 0.0f, 0.0f});
    add(render::makeRing(boundary + runOff, surroundRadius, 192), surround, at(Vec3(0.0f)), false);

    // Boundary rope lying on the grass.
    float ropeR = c.getFloat("field.ropeRadius", 0.05f);
    MaterialDesc rope = readMaterial(c, "field.rope", {{0.95f, 0.95f, 0.95f}, 0.6f, 0.0f, 0.0f, 0.0f});
    add(render::makeTorus(boundary, ropeR, 256, 8), rope, at(Vec3(0, ropeR, 0)), true);

    // 30-yard fielding circle: painted discs, merged into one mesh.
    MaterialDesc paint = readMaterial(c, "crease", {{0.96f, 0.96f, 0.96f}, 0.7f, 0.0f, 0.0f, 0.0f});
    {
        float radius = c.getFloat("field.innerCircleRadius", 27.43f);
        float spacing = c.getFloat("field.innerCircleMarkerSpacing", 3.0f);
        render::MeshData dot = render::makeDisc(c.getFloat("field.innerCircleMarkerRadius", 0.15f), 16);
        render::MeshData markers;
        for (Vec3 p : fieldingCirclePoints(radius, halfPitch, spacing))
            render::appendMesh(markers, dot, at(p + Vec3(0, kPaintY, 0)));
        add(markers, paint, at(Vec3(0.0f)), false);
    }

    // --- Pitch: 20.12 m (22 yards) between the stumps, 3.05 m wide, along Z. ---
    float pitchWidth = c.getFloat("pitch.width", 3.05f);
    float pitchOverrun = c.getFloat("pitch.overrun", 1.22f);  // prepared surface beyond each set of stumps
    MaterialDesc pitchMat = readMaterial(c, "pitch", {{0.78f, 0.70f, 0.50f}, 0.85f, 0.0f, 0.0f, 0.0f});
    add(render::makePlane(pitchWidth, scene.pitchLength + 2.0f * pitchOverrun), pitchMat,
            at(Vec3(0, kPitchY, 0)), false);

    // --- Creases (Law 7): bowling, popping and return creases at both ends, one mesh. ---
    {
        float lineW = c.getFloat("crease.lineWidth", 0.05f);
        float popping = c.getFloat("crease.poppingDistance", 1.22f);  // in front of the stumps
        float poppingLen = c.getFloat("crease.poppingLength", 3.66f);
        float bowlingLen = c.getFloat("crease.bowlingLength", 2.64f);
        float returnX = 0.5f * bowlingLen;                         // 1.32 m either side of middle stump
        float returnLen = c.getFloat("crease.returnLength", 2.44f);  // popping crease back to 1.22 m behind stumps
        render::MeshData bowling = render::makePlane(bowlingLen, lineW);
        render::MeshData poppingLine = render::makePlane(poppingLen, lineW);
        render::MeshData returnLine = render::makePlane(lineW, returnLen);
        render::MeshData lines;
        for (int end = 0; end < 2; ++end) {
            float sign = end == 0 ? 1.0f : -1.0f;
            float stumpZ = sign * halfPitch;
            render::appendMesh(lines, bowling, at(Vec3(0, kPaintY, stumpZ)));
            render::appendMesh(lines, poppingLine, at(Vec3(0, kPaintY, stumpZ - sign * popping)));
            // Return creases run from the popping crease back past the stumps.
            float returnCentreZ = stumpZ - sign * (popping - 0.5f * returnLen);
            render::appendMesh(lines, returnLine, at(Vec3(returnX, kPaintY, returnCentreZ)));
            render::appendMesh(lines, returnLine, at(Vec3(-returnX, kPaintY, returnCentreZ)));
        }
        add(lines, paint, at(Vec3(0.0f)), false);
    }

    // --- Wickets (Law 8): three stumps and two bails at each end. ---
    float stumpH = c.getFloat("stumps.height", 0.7112f);
    float stumpR = c.getFloat("stumps.radius", 0.0178f);
    float wicketW = c.getFloat("stumps.wicketWidth", 0.2286f);
    float bailLen = c.getFloat("bails.length", 0.1095f);
    float bailR = c.getFloat("bails.radius", 0.0065f);
    float bailRise = c.getFloat("bails.heightAboveStumps", 0.0127f);  // max protrusion above the stumps
    MaterialDesc wood = readMaterial(c, "stumps", {{0.93f, 0.88f, 0.75f}, 0.45f, 0.0f, 0.0f, 0.0f});
    MaterialDesc bailWood = readMaterial(c, "bails", wood);
    render::MeshData stumpMesh = render::makeCylinder(stumpR, stumpH, 24);
    render::MeshData bailMesh = render::makeCylinder(bailR, bailLen, 12);
    float spacing = 0.5f * wicketW - stumpR;  // centre of an outer stump from the middle stump
    // Bail cylinders are built along +Y; lay them along X.
    math::Quat alongX = math::fromAxisAngle(math::Vec3(0, 0, 1), -math::kHalfPi);
    float bailY = stumpH + bailRise - bailR;  // top of bail at the allowed height
    for (int end = 0; end < 2; ++end) {
        float z = (end == 0 ? 1.0f : -1.0f) * halfPitch;
        for (int s = -1; s <= 1; ++s)
            add(stumpMesh, wood, at(Vec3(static_cast<float>(s) * spacing, kPitchY, z)), true);
        for (int b = 0; b < 2; ++b) {
            float centreX = (b == 0 ? -0.5f : 0.5f) * spacing;  // each bail spans middle to outer stump
            add(bailMesh, bailWood, at(Vec3(centreX - 0.5f * bailLen, kPitchY + bailY, z), alongX), true);
        }
    }

    // --- Ball (Law 4): men's ball, 22.4-22.9 cm circumference. ---
    {
        float circumference = c.getFloat("ball.circumference", 0.2265f);
        float radius = circumference / math::kTwoPi;
        MaterialDesc leather = readMaterial(c, "ball", {{0.55f, 0.06f, 0.05f}, 0.35f, 0.0f, 0.0f, 0.0f});
        leather.kind = render::MaterialKind::CricketBall;
        Vec3 pos = c.getVec3("ball.restPosition", Vec3(0.15f, 0.0f, 5.0f));
        // The shader's seam is the object's x = 0 circle, so with no rotation
        // it stands upright along the pitch, as when bowled.
        scene.ball = add(render::makeSphere(radius, 48, 24), leather,
                             at(pos + Vec3(0, kPitchY + radius, 0)), true);
    }

    // --- Probe that follows the fixed-step simulation (interpolated). ---
    MaterialDesc probeMat = readMaterial(c, "probe", {{0.8f, 0.1f, 0.1f}, 0.35f, 0.0f, 0.0f, 0.0f});
    scene.probe = add(render::makeSphere(0.25f, 24, 12), probeMat, at(Vec3(0.0f)), true);
    w.registry().emplace<FollowsProbe>(scene.probe);

    return scene;
}

void destroyTestScene(World& w, render::Renderer& r, TestScene& scene) {
    for (entt::entity e : scene.entities) w.despawn(r, e);
    scene = TestScene{};
}

} // namespace game
