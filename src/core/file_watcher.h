#pragma once
// Watches a directory for changed files (efsw underneath; its callbacks run on
// a background thread). The main loop calls poll() once per frame to collect
// files that changed and have since been quiet for a short moment, so an
// editor's multi-step save is reported once. Dev-tool only: allocates.

#include <memory>
#include <string>
#include <vector>

namespace core {

class FileWatcher {
public:
    FileWatcher();
    ~FileWatcher();

    // Starts watching `dir` (non-recursive). Returns false if it can't.
    bool watch(const std::string& dir);

    // Appends file names (not paths) changed since the last call.
    void poll(std::vector<std::string>& changed);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace core
