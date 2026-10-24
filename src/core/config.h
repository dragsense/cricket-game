#pragma once
// Read-only access to config/*.json tunables (parsed with nlohmann/json).
// Values are looked up by dotted path with a fallback, so a missing key never
// crashes and the calling code documents its default. Read values once at
// startup and cache them; lookups allocate and must stay out of the sim loop.

#include "math/math.h"

#include <nlohmann/json.hpp>

#include <string>
#include <string_view>

namespace core {

class Config {
public:
    // Loads and parses a file. Logs and returns false on failure (the config
    // then just yields fallbacks). Accepts // and /* */ comments.
    bool load(const std::string& path);

    const nlohmann::json& root() const { return root_; }

    float getFloat(std::string_view path, float fallback) const;
    int getInt(std::string_view path, int fallback) const;
    bool getBool(std::string_view path, bool fallback) const;
    std::string getString(std::string_view path, const std::string& fallback) const;
    math::Vec3 getVec3(std::string_view path, math::Vec3 fallback) const;

private:
    // Returns nullptr if the path does not exist. "a.b.c" style paths.
    const nlohmann::json* find(std::string_view path) const;

    nlohmann::json root_ = nlohmann::json::object();
};

} // namespace core
