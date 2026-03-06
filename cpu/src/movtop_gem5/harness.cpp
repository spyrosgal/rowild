/*
 * MIT License
 *
 * Copyright (c) 2023 Carnegie Mellon University
 *
 * This file is part of RoWild.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "args.h"
#include "log.h"
#include "move_controller.h"
#include "zsim_hooks.h"
#include <fstream>
#include <math.h>
#include <string>
#include <vector>

#include "hw_contracts.h"

std::vector<STATE> theLog;

void loadLog(std::string inputFile) {
    // std::ifstream logFile;
    // logFile.open(inputFile);
    // assert(logFile.good());

    // std::string line;
    // std::getline(logFile, line);
    // assert(line == "X Y Theta");

    // double x, y, theta;
    // while (logFile >> x >> y >> theta) {
    //     theLog.push_back({x, y, theta});
    // }

    // logFile.close();

    double lower_bound = 0;
    double upper_bound = 20;
    double theta_lower_bound = -3;
    double theta_upper_bound = 3;
    std::uniform_real_distribution<double> unif(lower_bound,upper_bound);
    std::uniform_real_distribution<double> unif_theta(theta_lower_bound,theta_upper_bound);
    std::default_random_engine re;
    for(int i = 0; i < 400000; i++) {
        if(!(i%10000)) std::cout << i << std::endl;
        theLog.push_back({unif(re), unif(re), unif_theta(re)});
    }
}

int main(int argc, const char **argv) {
    // using args::KVArg;
    // using args::Parser;

    // Parser parser(argv[0], argc, argv, false);
    // KVArg<int> numRunsArg(parser, "num_runs", "", "Number of repetitions");
    // KVArg<int> m5exitArg(parser, "m5_exit", "", "Whether m5_exit should be called before and after the benchmark");
    // KVArg<int> deadlinesArg(parser, "deadlines", "", "Whether we should actually track deadlines");
    // KVArg<std::string> logFileArg(parser, "log", "", "Input log file");
    // KVArg<double> dtArg(parser, "dt", "", "Time step");
    // KVArg<double> thresholdArg(parser, "threshold", "", "Close enough to goal");
    // KVArg<double> maxLinSpeedArg(parser, "max-lin-speed", "",
    //                              "Maximum linear speed");
    // KVArg<double> maxAngSpeedArg(parser, "max-ang-speed", "",
    //                              "Maximum angular speed");
    // KVArg<std::string> outArg(parser, "output", "", "Output log file");

    // if (!parser.parse()) assert(false);

    // assert_msg(logFileArg.found(), "Input file is not provided");

    std::string logFileName = "./rowild/cpu/src/movtop_gem5/movtop_gem5.out";
    // const int num_runs = numRunsArg.found() ? numRunsArg.value() : 2;
    // const int should_m5_exit = m5exitArg.found() ? m5exitArg.value() : 1;
    // const int deadlines = deadlinesArg.found() ? deadlinesArg.value() : 1;
    // double dt = dtArg.found() ? dtArg.value() : 0.01;
    // double threshold = thresholdArg.found() ? thresholdArg.value() : 0.001;
    // double maxLinSpeed = maxLinSpeedArg.found() ? maxLinSpeedArg.value() : 15.0;
    // double maxAngSpeed = maxAngSpeedArg.found() ? maxAngSpeedArg.value() : 7.0;
    // std::string outputFile = outArg.found() ? outArg.value() : "/dev/null";

    // const int num_runs = (argc > 1) ? atoi(argv[1]) : 2;
    // const int should_m5_exit = (argc > 2) ? atoi(argv[2]) : 1;
    // const int deadlines = (argc > 3) ? atoi(argv[3]) : 1;
    const int num_runs = 100;
    const int should_m5_exit = 1;
    const int deadlines = 1;
    double dt = 0.1;
    double threshold = 0.01;
    double maxLinSpeed = 15.0;
    double maxAngSpeed = 7.0;

    loadLog(logFileName);
    assert(theLog.size() % 2 == 0);

    MoveController *ctrl =
        new MoveController(dt, threshold, maxLinSpeed, maxAngSpeed);
    std::vector<std::vector<std::pair<double, double>>> trajLog;

    if(should_m5_exit) m5_exit(0);
    
    uint64_t cid = 0;

    if(deadlines) {
        cid = hwc_create_contract();
        hwc_add_core(cid);
        hwc_set_deadline(cid, 200);
        hwc_update_linreg(cid, 0, 3.814825e-01);
        hwc_update_linreg(cid, 1, 8.271875e-01);
        hwc_update_linreg(cid, 2, 2166.905);
    }

    // ROI begins
    zsim_roi_begin();
    for (int i = 0; i < 200 * num_runs; i += 200) {
        if(should_m5_exit) m5_exit(0);
        if(deadlines) hwc_start_roi(cid);

        for(int j = 0; j < 200; j += 2) {
            STATE start = theLog[i + j];
            STATE goal = theLog[i + j + 1];
            std::vector<std::pair<double, double>> traj =
                ctrl->getTrajectory(start, goal);
            trajLog.push_back(traj);
        }

        if(deadlines) hwc_end_roi(cid);
        if(should_m5_exit) m5_exit(0);
    }
    zsim_roi_end();
    // ROI ends

    if(deadlines) {
        hwc_remove_core(cid);
        hwc_delete_contract(cid);
    }

    // Write the output trajectory
    // std::ofstream outTraj;
    // outTraj.open(outputFile);
    // for (std::vector<std::pair<double, double>> traj : trajLog) {
    //     for (std::pair<double, double> t : traj) {
    //         outTraj << t.first << " " << t.second << std::endl;
    //     }
    //     outTraj << "----------" << std::endl;
    // }
    // outTraj.close();

    return 0;
}
