#include "core/config.h"

#include "core/log.h"

#include <fstream>

namespace core {

bool Config::load(const std::string& path) {
    std::ifstream file(path);
    if (!file) {
        LOG_ERROR("config: cannot open {}", path);
        return false;
    }
    try {
        root_ = nlohmann::json::parse(file, nullptr, /*allow_exceptions=*/true, /*ignore_comments=*/true);
    } catch (const nlohmann::json::parse_error& e) {
        LOG_ERROR("config: {}: {}", path, e.what());
        root_ = nlohmann::json::object();
        return false;
    }
    LOG_INFO("config: loaded {}", path);
    return true;
}

const nlohmann::json* Config::find(std::string_view path) const {
    const nlohmann::json* node = &root_;
    while (!path.empty()) {
        size_t dot = path.find('.');
        std::string key(path.substr(0, dot));
        if (!node->is_object()) return nullptr;
        auto it = node->find(key);
        if (it == node->end()) return nullptr;
        node = &*it;
        if (dot == std::string_view::npos) break;
        path.remove_prefix(dot + 1);
    }
    return node;
}

float Config::getFloat(std::string_view path, float fallback) const {
    const auto* v = find(path);
    return v && v->is_number() ? v->get<float>() : fallback;
}

int Config::getInt(std::string_view path, int fallback) const {
    const auto* v = find(path);
    return v && v->is_number() ? v->get<int>() : fallback;
}

bool Config::getBool(std::string_view path, bool fallback) const {
    const auto* v = find(path);
    return v && v->is_boolean() ? v->get<bool>() : fallback;
}

std::string Config::getString(std::string_view path, const std::string& fallback) const {
    const auto* v = find(path);
    return v && v->is_string() ? v->get<std::string>() : fallback;
}

math::Vec3 Config::getVec3(std::string_view path, math::Vec3 fallback) const {
    const auto* v = find(path);
    if (!v || !v->is_array() || v->size() != 3) return fallback;
    return {(*v)[0].get<float>(), (*v)[1].get<float>(), (*v)[2].get<float>()};
}

} // namespace core
