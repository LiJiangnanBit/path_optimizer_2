//
// Created by ljn on 21-10-13.
//
#include "tools/time_recorder.h"
#include "tools/tools.hpp"
#include "glog/logging.h"

namespace PathOptimizationNS {

void TimeRecorder::recordTime(const std::string &name) {
    time_stamps_.emplace_back(std::clock());
    names_.emplace_back(name);
}

void TimeRecorder::printTime() const {
    if (time_stamps_.size() <= 1) {
        LOG(WARNING) << "time stamps size not enough!";
        return;
    }
    for (size_t i = 0; i < time_stamps_.size() - 1; ++i) {
        LOG(INFO) << names_[i] << " cost " << time_ms(time_stamps_[i], time_stamps_[i + 1]) << " ms.";
    }
    if (time_stamps_.size() > 2) {
        LOG(INFO) << "Total time cost: " << time_ms(time_stamps_.front(), time_stamps_.back()) << " ms.";
    }
}

void TimeRecorder::clear() {
    time_stamps_.clear();
    names_.clear();
}

}

