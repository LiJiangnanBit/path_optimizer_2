//
// Created by ljn on 21-10-13.
//

#ifndef PATH_OPTIMIZER_2_INCLUDE_TOOLS_TIME_RECORDER_H_
#define PATH_OPTIMIZER_2_INCLUDE_TOOLS_TIME_RECORDER_H_
#include <vector>
#include <string>
#include <ctime>
#include "glog/logging.h"

namespace PathOptimizationNS {

class TimeRecorder {
 public:
    TimeRecorder(const std::string& title) : title_(title) {}
    void recordTime(const std::string &name);
    void printTime() const;
    void clear();

 private:
    std::string title_;
    std::vector<std::string> names_;
    std::vector<clock_t> time_stamps_;
};

}

#endif //PATH_OPTIMIZER_2_INCLUDE_TOOLS_TIME_RECORDER_H_
