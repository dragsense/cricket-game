#include "core/file_watcher.h"

#include "core/log.h"

#include <efsw/efsw.hpp>

#include <chrono>
#include <mutex>
#include <unordered_map>

namespace core {

namespace {
using Clock = std::chrono::steady_clock;
constexpr auto kQuietTime = std::chrono::milliseconds(150);  // editors often write a file several times
} // namespace

struct FileWatcher::Impl final : efsw::FileWatchListener {
    efsw::FileWatcher watcher;
    std::mutex mutex;
    std::unordered_map<std::string, Clock::time_point> pending;  // file name -> last change

    void handleFileAction(efsw::WatchID, const std::string&, const std::string& filename, efsw::Action action,
                          std::string) override {
        if (action != efsw::Actions::Modified && action != efsw::Actions::Add && action != efsw::Actions::Moved)
            return;
        std::lock_guard<std::mutex> lock(mutex);
        pending[filename] = Clock::now();
    }
};

FileWatcher::FileWatcher() : impl_(std::make_unique<Impl>()) {}

FileWatcher::~FileWatcher() = default;

bool FileWatcher::watch(const std::string& dir) {
    efsw::WatchID id = impl_->watcher.addWatch(dir, impl_.get(), false);
    if (id < 0) {
        LOG_WARN("file watcher: cannot watch {}: {}", dir, efsw::Errors::Log::getLastErrorLog());
        return false;
    }
    impl_->watcher.watch();
    LOG_INFO("file watcher: watching {}", dir);
    return true;
}

void FileWatcher::poll(std::vector<std::string>& changed) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    auto now = Clock::now();
    for (auto it = impl_->pending.begin(); it != impl_->pending.end();) {
        if (now - it->second >= kQuietTime) {
            changed.push_back(it->first);
            it = impl_->pending.erase(it);
        } else {
            ++it;
        }
    }
}

} // namespace core
