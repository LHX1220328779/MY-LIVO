#pragma once
// Wall-clock section timing for the per-frame LIO/VIO backend.
// Laps accumulate in a process-wide table. With debug.print_frame_timing on, the table is printed every
// kReportEvery LIO frames as the mean time per LIO frame of each section (VIO sections run once per camera
// frame and are normalised by the same LIO frame count). A lap costs one steady_clock read.
#include <chrono>
#include <cstdio>
#include <map>
#include <string>

namespace frame_profile
{
constexpr int kReportEvery = 50;

class Table
{
public:
  static Table &instance()
  {
    static Table table;
    return table;
  }

  void add(const char *name, double seconds)
  {
    Section &s = sections_[name];
    s.total += seconds;
    if (seconds > s.max) s.max = seconds;
  }

  // Called once per LIO frame.
  void endFrame(bool print)
  {
    if (++frames_ < kReportEvery) return;
    if (print)
    {
      double sum = 0.0;
      std::printf("\033[1;34m+------------------------------------------------------------+\033[0m\n");
      std::printf("\033[1;34m| Frame section (mean ms per LIO frame, %3d frames) | max ms |\033[0m\n", frames_);
      std::printf("\033[1;34m+------------------------------------------------------------+\033[0m\n");
      for (const auto &kv : sections_)
      {
        const double mean_ms = 1000.0 * kv.second.total / frames_;
        sum += mean_ms;
        std::printf("\033[1;36m| %-40s | %8.3f | %6.2f |\033[0m\n", kv.first.c_str(), mean_ms, 1000.0 * kv.second.max);
      }
      std::printf("\033[1;34m+------------------------------------------------------------+\033[0m\n");
      std::printf("\033[1;32m| %-40s | %8.3f |\033[0m\n", "sum of sections", sum);
      std::printf("\033[1;34m+------------------------------------------------------------+\033[0m\n");
      std::fflush(stdout);  // the mapper is stopped by signal; do not leave the table in the stdio buffer
    }
    sections_.clear();
    frames_ = 0;
  }

private:
  struct Section
  {
    double total = 0.0;
    double max = 0.0;
  };
  std::map<std::string, Section> sections_;
  int frames_ = 0;
};

class Stopwatch
{
public:
  Stopwatch() : last_(Clock::now()) {}

  void restart() { last_ = Clock::now(); }

  void lap(const char *name)
  {
    const Clock::time_point now = Clock::now();
    Table::instance().add(name, std::chrono::duration<double>(now - last_).count());
    last_ = now;
  }

private:
  using Clock = std::chrono::steady_clock;
  Clock::time_point last_;
};
class Scoped
{
public:
  explicit Scoped(const char *name) : name_(name), start_(Clock::now()) {}
  ~Scoped() { Table::instance().add(name_, std::chrono::duration<double>(Clock::now() - start_).count()); }

private:
  using Clock = std::chrono::steady_clock;
  const char *name_;
  Clock::time_point start_;
};
}  // namespace frame_profile
