// Rolling level history for the History screen.
//
// The meter hands over the A-weighted mean square of each whole second.
// Seconds are grouped into bars of `seconds_per_bar`, and each bar's level is
// the energy average (Leq) of its seconds. The newest bar is still filling
// and is included as soon as it holds one second, so the graph moves every
// second rather than once per bar.
//
// With each second the caller also passes the rolling LAeq (as a mean
// square); each bar keeps the value from its latest second, so the graph can
// mark what LAeq was at that point.
//
// With the defaults (90 bars x 10 s) the history covers the last 15 minutes.
//
// No ESP-IDF dependencies: unit-tested on the host by test/host_test.cpp.

#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace slm {

class LevelHistory {
public:
    explicit LevelHistory(size_t bars = 90, size_t seconds_per_bar = 10)
        : bars_(bars), seconds_per_bar_(seconds_per_bar), done_(bars > 1 ? bars - 1 : 1, 0.0), done_leq_(done_.size(), 0.0)
    {
    }

    size_t bars() const { return bars_; }
    size_t seconds_per_bar() const { return seconds_per_bar_; }

    // Adds one second's mean square, and the rolling LAeq mean square at the
    // end of it. Bumps version().
    void push_second(double ms, double leq_ms = 0.0)
    {
        cur_sum_ += ms;
        cur_leq_ = leq_ms;
        if (++cur_count_ == seconds_per_bar_) {
            // Bar complete: keep the last bars-1 complete bars, so with the
            // filling bar there are never more than `bars`.
            done_[head_] = cur_sum_ / double(seconds_per_bar_);
            done_leq_[head_] = cur_leq_;
            head_ = (head_ + 1) % done_.size();
            if (done_count_ < done_.size()) ++done_count_;
            cur_sum_ = 0;
            cur_count_ = 0;
        }
        ++version_;
    }

    // Changes whenever the history does; the UI copies only when it moves.
    uint32_t version() const { return version_; }

    // Copies bar mean squares, oldest first, into out (room for bars()), and
    // if leq_out is given the LAeq mean square at each bar's latest second.
    // The last entry is the bar still filling, when it has any seconds.
    // Returns the number of entries written.
    size_t copy(double *out, double *leq_out = nullptr) const
    {
        size_t n = 0;
        const size_t cap = done_.size();
        // At most bars-1 complete bars are kept, so the filling bar always fits.
        const size_t first = (head_ + cap - done_count_) % cap;
        for (size_t i = 0; i < done_count_; ++i, ++n) {
            out[n] = done_[(first + i) % cap];
            if (leq_out) leq_out[n] = done_leq_[(first + i) % cap];
        }
        if (cur_count_ > 0) {
            out[n] = cur_sum_ / double(cur_count_);
            if (leq_out) leq_out[n] = cur_leq_;
            ++n;
        }
        return n;
    }

private:
    size_t bars_, seconds_per_bar_;
    std::vector<double> done_;      // ring of complete bars (bars - 1 of them)
    std::vector<double> done_leq_;  // LAeq at the end of each complete bar
    size_t head_ = 0, done_count_ = 0;
    double cur_sum_ = 0;        // the bar being filled
    double cur_leq_ = 0;        // LAeq at its latest second
    size_t cur_count_ = 0;
    uint32_t version_ = 0;
};

}  // namespace slm
