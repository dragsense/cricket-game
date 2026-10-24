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
    return m;
}

math::Transform at(Vec3 pos, math::Quat rot = math::identityQuat()) {
    math::Transform t;
    t.position = pos;
    t.rotation = rot;
    return t;
}

} // namespace

TestScene buildTestScene(World& w, render::Renderer& r, const core::Config& c) {
    TestScene scene;

    // --- Field: a grass disc out to the boundary, with a wider surround. ---
    float fieldRadius = c.getFloat("field.radius", 70.0f);
    MaterialDesc grass = readMaterial(c, "field.grass", {{0.25f, 0.5f, 0.18f}, 0.9f, 0.0f, 4.0f, 0.12f});
    w.spawn(r, render::makeDisc(fieldRadius, 128), grass, at(Vec3(0.0f)), false);

    MaterialDesc surround = readMaterial(c, "field.surround", {{0.2f, 0.4f, 0.15f}, 0.95f, 0.0f, 0.0f, 0.0f});
    float surroundSize = c.getFloat("field.surroundSize", 400.0f);
    w.spawn(r, render::makePlane(surroundSize, surroundSize), surround, at(Vec3(0, -0.01f, 0)), false);

    // --- Pitch: 22 yards between the stumps, laid along Z. ---
    scene.pitchLength = c.getFloat("pitch.length", 20.12f);
    float pitchWidth = c.getFloat("pitch.width", 3.05f);
    float pitchOverrun = c.getFloat("pitch.overrun", 1.22f);  // prepared surface beyond each set of stumps
    MaterialDesc pitchMat = readMaterial(c, "pitch", {{0.76f, 0.68f, 0.48f}, 0.85f, 0.0f, 0.0f, 0.0f});
    w.spawn(r, render::makePlane(pitchWidth, scene.pitchLength + 2.0f * pitchOverrun), pitchMat, at(Vec3(0, 0.004f, 0)), false);

    // --- Creases (painted lines, 5 cm wide). ---
    MaterialDesc paint = readMaterial(c, "crease", {{0.95f, 0.95f, 0.95f}, 0.7f, 0.0f, 0.0f, 0.0f});
    float lineW = c.getFloat("crease.lineWidth", 0.05f);
    float popping = c.getFloat("crease.poppingDistance", 1.22f);
    float creaseLen = c.getFloat("crease.poppingLength", 3.66f);
    float bowlingLen = c.getFloat("crease.bowlingLength", 2.64f);
    render::MeshData bowlingCrease = render::makePlane(bowlingLen, lineW);
    render::MeshData poppingCrease = render::makePlane(creaseLen, lineW);
    for (int end = 0; end < 2; ++end) {
        float sign = end == 0 ? 1.0f : -1.0f;
        float stumpZ = sign * 0.5f * scene.pitchLength;
        w.spawn(r, bowlingCrease, paint, at(Vec3(0, 0.006f, stumpZ)), false);
        w.spawn(r, poppingCrease, paint, at(Vec3(0, 0.006f, stumpZ - sign * popping)), false);
    }

    // --- Stumps and bails at both ends. ---
    float stumpH = c.getFloat("stumps.height", 0.711f);
    float stumpR = c.getFloat("stumps.radius", 0.0175f);
    float wicketW = c.getFloat("stumps.wicketWidth", 0.2286f);
    MaterialDesc wood = readMaterial(c, "stumps", {{0.93f, 0.88f, 0.75f}, 0.45f, 0.0f, 0.0f, 0.0f});
    render::MeshData stumpMesh = render::makeCylinder(stumpR, stumpH, 16);
    float spacing = 0.5f * wicketW - stumpR;  // centre of outer stump from middle stump
    render::MeshData bailMesh = render::makeBox(Vec3(0.02f, 0.02f, spacing + 0.01f));
    for (int end = 0; end < 2; ++end) {
        float z = (end == 0 ? 0.5f : -0.5f) * scene.pitchLength;
        for (int s = -1; s <= 1; ++s)
            w.spawn(r, stumpMesh, wood, at(Vec3(static_cast<float>(s) * spacing, 0, z)), true);
        math::Quat across = math::fromAxisAngle(math::kUp, math::kHalfPi);  // bails lie along X
        for (int b = 0; b < 2; ++b)
            w.spawn(r, bailMesh, wood, at(Vec3((b == 0 ? -0.5f : 0.5f) * spacing, stumpH + 0.01f, z), across), true);
    }

    // --- Human-sized capsules to judge scale. ---
    float humanH = c.getFloat("human.height", 1.8f);
    float humanR = c.getFloat("human.radius", 0.22f);
    MaterialDesc kit = readMaterial(c, "human", {{0.92f, 0.92f, 0.9f}, 0.7f, 0.0f, 0.0f, 0.0f});
    render::MeshData humanMesh = render::makeCapsule(humanR, humanH, 24, 8);
    const Vec3 spots[] = {
        {0.6f, 0, 0.5f * scene.pitchLength - 1.3f},  // batter at the crease
        {0.0f, 0, 0.5f * scene.pitchLength + 2.5f},  // keeper
        {1.2f, 0, -0.5f * scene.pitchLength - 1.0f}, // bowler / non-striker end
        {18.0f, 0, -6.0f},                           // a fielder
    };
    for (Vec3 p : spots) w.spawn(r, humanMesh, kit, at(p), true);

    // --- 1 m reference cube. ---
    MaterialDesc grey = readMaterial(c, "referenceCube", {{0.6f, 0.6f, 0.62f}, 0.5f, 0.0f, 0.0f, 0.0f});
    w.spawn(r, render::makeBox(Vec3(1.0f)), grey, at(Vec3(4.0f, 0.5f, 0.0f)), true);

    // --- Probe that follows the fixed-step simulation (interpolated). ---
    MaterialDesc probeMat = readMaterial(c, "probe", {{0.8f, 0.1f, 0.1f}, 0.35f, 0.0f, 0.0f, 0.0f});
    scene.probe = w.spawn(r, render::makeSphere(0.25f, 24, 12), probeMat, at(Vec3(0.0f)), true);
    w.registry().emplace<FollowsProbe>(scene.probe);

    return scene;
}

} // namespace game
