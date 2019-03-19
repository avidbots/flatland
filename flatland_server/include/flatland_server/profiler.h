#ifndef FLATLAND_PROFILER_H
#define FLATLAND_PROFILER_H

#define PROFILER_ON false
#define PROFILER_OUTPUT_PATH "/tmp/flatland_profile_output.log"

#if PROFILER_ON
#define START_PROFILE(timekeeper, name) timekeeper.profiler_.get(name).start()
#define END_PROFILE(timekeeper, name) timekeeper.profiler_.get(name).end()
#define PRINT_ALL_PROFILES(timekeeper) timekeeper.profiler_.print()
#else
#define START_PROFILE(timekeeper, name)
#define END_PROFILE(timekeeper, name)
#define PRINT_ALL_PROFILES(timekeeper)
#endif

#include <cassert>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <unordered_map>

namespace flatland_server {

class Profile {
 public:
  Profile()
      : lapse_count_(0),
        started_(false),
        start_time_(std::chrono::high_resolution_clock::now()),
        total_duration_(std::chrono::high_resolution_clock::duration::zero()) {}

  void start() {
    started_ = true;
    start_time_ = std::chrono::high_resolution_clock::now();
  }

  void end() {
    assert(started_);
    lapse_count_++;
    total_duration_ += std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::high_resolution_clock::now() - start_time_);
  }

  int getLapseCount() const { return lapse_count_; }

  int64_t getTotalDuration() const {
    return std::chrono::duration_cast<std::chrono::microseconds>(
               total_duration_)
        .count();
  }

 private:
  int lapse_count_;
  bool started_;
  std::chrono::high_resolution_clock::time_point start_time_;
  std::chrono::high_resolution_clock::duration total_duration_;
};

class Profiler {
 public:
  Profile& get(const std::string& profile_name) {
    auto p = profiles_.find(profile_name);
    if (p == profiles_.end()) {
      auto profile = profiles_.emplace(std::piecewise_construct,
                                       std::forward_as_tuple(profile_name),
                                       std::forward_as_tuple());
      return profile.first->second;
    }
    return p->second;
  }

  void print() {
    std::ofstream out(PROFILER_OUTPUT_PATH);

    out << std::left << std::setw(60) << "Profile" << std::left << std::setw(20)
        << "Lapse Count" << std::left << std::setw(20) << "Total Duration (us)"
        << std::left << std::setw(20) << "Average Duration (us)" << std::endl;

    for (const auto& prof : profiles_) {
      out << std::left << std::setw(60) << prof.first << std::left
          << std::setw(20) << prof.second.getLapseCount() << std::left
          << std::setw(20) << prof.second.getTotalDuration() << std::left
          << std::setw(20) << std::setprecision(10)
          << ((prof.second.getLapseCount() == 0)
                  ? 0
                  : (prof.second.getTotalDuration() /
                     prof.second.getLapseCount()))
          << std::endl;
    }
    out.close();
  }

 private:
  std::unordered_map<std::string, Profile> profiles_;
};

}  // namespace flatland_server

#endif  // FLATLAND_PROFILER_H
