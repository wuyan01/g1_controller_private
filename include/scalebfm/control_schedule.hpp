#pragma once

#include <array>
#include <chrono>
#include <cstddef>
#include <sstream>
#include <stdexcept>
#include <string>

namespace scalebfm {

struct Timing {
  double gap_ms = 0.;
  double lateness_ms = 0.;
  double window_delay_ms = 0.;
  double work_ms = 0.;
  bool first_tick = true;
  bool noticed = false;
  // Whether begin accepted this attempt; finish can subsequently fault.
  bool accepted = false;
};

// Nominal 50 Hz sampling with bounded host wake jitter. Each accepted tick
// samples exactly once; lateness never creates compressed catch-up samples.
// This is not a real-time guarantee or a substitute for state/action age guards.
class ControlSchedule {
 public:
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;
  using Nanoseconds = std::chrono::nanoseconds;

  inline static constexpr Nanoseconds kPeriod = std::chrono::milliseconds(20);
  inline static constexpr Nanoseconds kMaxSampleGap = std::chrono::milliseconds(40);
  inline static constexpr Nanoseconds kNoticeLateness = std::chrono::milliseconds(5);
  inline static constexpr Nanoseconds kWindow = std::chrono::seconds(1);
  inline static constexpr Nanoseconds kWindowBudget = std::chrono::milliseconds(50);
  inline static constexpr Nanoseconds kWorkDeadline = std::chrono::milliseconds(20);

  explicit ControlSchedule(TimePoint first_due = Clock::now()) { reset(first_due); }

  void reset(TimePoint first_due) {
    next_ = first_due;
    last_start_ = TimePoint{};
    last_finish_ = TimePoint{};
    have_tick_ = false;
    finished_ = false;
    faulted_ = false;
    head_ = count_ = 0;
    delay_sum_ = Nanoseconds::zero();
    timing_ = Timing{};
  }

  Timing begin(TimePoint actual_start) {
    if (faulted_) throw std::runtime_error("Control schedule fault is latched; reset required");
    timing_ = Timing{};
    timing_.first_tick = !have_tick_;
    const auto gap = have_tick_ ? ns(actual_start - last_start_) : Nanoseconds::zero();
    const auto lateness = actual_start > next_ ? ns(actual_start - next_) : Nanoseconds::zero();
    timing_.gap_ms = ms(gap);
    timing_.lateness_ms = ms(lateness);
    timing_.noticed = lateness > kNoticeLateness;
    timing_.window_delay_ms = ms(delay_sum_ + lateness);
    if (have_tick_ && !finished_) fail("Control tick began before preceding work finished");
    if (have_tick_ && actual_start <= last_start_)
      fail("Control start timestamp did not advance");
    if (actual_start < next_)
      fail("Control tick began before its next wake; catch-up inference is prohibited");

    // The rolling interval is (actual_start - 1 s, actual_start]. Charge all
    // positive wake delays, including ordinary sub-notice scheduler overhead.
    while (count_ && actual_start - delays_[head_].at >= kWindow) {
      delay_sum_ -= delays_[head_].amount;
      head_ = (head_ + 1) % delays_.size();
      --count_;
    }
    timing_.window_delay_ms = ms(delay_sum_ + lateness);
    if (have_tick_ && gap > kMaxSampleGap)
      fail("Control sample gap exceeded 40 ms");
    if (delay_sum_ + lateness > kWindowBudget)
      fail("Repeated control wake overruns exceeded 50 ms in the rolling 1 s window");
    if (lateness > Nanoseconds::zero()) {
      // Accepted starts are at least 20 ms apart, so at most 50 delay samples
      // can occupy a one-second window. Fixed storage avoids tick allocations.
      if (count_ == delays_.size()) fail("Control wake delay accounting capacity exceeded");
      delays_[(head_ + count_) % delays_.size()] = {actual_start, lateness};
      ++count_;
      delay_sum_ += lateness;
    }
    last_start_ = actual_start;
    last_finish_ = actual_start;
    next_ = actual_start + kPeriod;
    have_tick_ = true;
    finished_ = false;
    timing_.accepted = true;
    return timing_;
  }

  // May be called again after small timing/trace bookkeeping; each call checks
  // the entire elapsed tick, not only inference or time since the last finish.
  void finish(TimePoint work_end) {
    if (faulted_) throw std::runtime_error("Control schedule fault is latched; reset required");
    if (!have_tick_) fail("Control work finished without an accepted tick");
    const auto work = ns(work_end - last_start_);
    timing_.work_ms = ms(work);
    if (work_end < last_start_ || (finished_ && work_end < last_finish_))
      fail("Control work timestamp moved backwards");
    if (work > kWorkDeadline) fail("Complete control work exceeded 20 ms");
    last_finish_ = work_end;
    finished_ = true;
  }

  TimePoint next_wake() const { return next_; }
  const Timing& timing() const { return timing_; }
  bool faulted() const { return faulted_; }

 private:
  struct Delay { TimePoint at{}; Nanoseconds amount{}; };
  template <class Duration> static Nanoseconds ns(Duration value) {
    return std::chrono::duration_cast<Nanoseconds>(value);
  }
  static double ms(Nanoseconds value) {
    return std::chrono::duration<double, std::milli>(value).count();
  }
  [[noreturn]] void fail(const char* reason) {
    faulted_ = true;
    std::ostringstream out;
    out << reason << " (gap_ms=" << timing_.gap_ms
        << ", lateness_ms=" << timing_.lateness_ms
        << ", window_delay_ms=" << timing_.window_delay_ms
        << ", work_ms=" << timing_.work_ms << ')';
    throw std::runtime_error(out.str());
  }

  TimePoint next_{}, last_start_{}, last_finish_{};
  bool have_tick_ = false, finished_ = false, faulted_ = false;
  std::array<Delay, 64> delays_{};
  std::size_t head_ = 0, count_ = 0;
  Nanoseconds delay_sum_{};
  Timing timing_{};
};

}  // namespace scalebfm
