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
#include "ekf.h"
#include "fast.h"
#include "graph_based.h"
#include "log.h"
#include "slam.h"
#include "zsim_hooks.h"
#include <fstream>
#include <iomanip>
#include <string>
#include <vector>
#include "gem5/m5ops.h"
#include "hw_contracts.h"

std::vector<std::vector<double>> readLog(std::string inputLogFile) {
    std::ifstream logFile;
    logFile.open(inputLogFile);
    assert(logFile.good());

    std::vector<std::vector<double>> log;

    std::string line;
    while (std::getline(logFile, line)) {
        std::stringstream entrySS(line);

        std::vector<double> entry;
        double e;
        while (entrySS >> e) {
            entry.push_back(e);
        }

        log.push_back(entry);
    }
    logFile.close();

    return log;
}

int main(int argc, const char **argv) {
    using args::KVArg;
    using args::Parser;

    char *msg = "Hello from slam_gem5!\n";
    m5_write_file((void *) msg, strlen(msg), 0, "hello_slam.txt");
    std::cout << "SLAM Gem5 Harness" << std::endl;

    Parser parser(argv[0], argc, argv, false);
    KVArg<int> numRunsArg(parser, "num_runs", "", "Number of repetitions");
    KVArg<int> m5exitArg(parser, "m5_exit", "", "Whether m5_exit should be called before and after the benchmark");
    KVArg<int> deadlinesArg(parser, "deadlines", "", "Whether we should actually track deadlines");
    KVArg<std::string> inputArg(parser, "log", "", "Input log file");
    KVArg<std::string> algorithmArg(parser, "alg", "",
                                    "SLAM algorithm [EKF Fast GraphBased]");
    KVArg<int> particlesArg(parser, "particles", "",
                            "Number of particles with FastSLAM");
    KVArg<std::string> outputArg(parser, "output", "", "Output log file");

    if (!parser.parse()) assert(false);

    assert_msg(inputArg.found(), "Input log file is not provided");
    assert_msg(algorithmArg.found(), "Input SLAM algorithm is not provided");

    const int num_runs = numRunsArg.found() ? numRunsArg.value() : 10000000;
    const int should_m5_exit = m5exitArg.found() ? m5exitArg.value() : 0;
    const int deadlines = deadlinesArg.found() ? deadlinesArg.value() : 1;
    std::string inputFile = inputArg.value();
    const char *slamAlgorithm = algorithmArg.value().c_str();
    int numParticles = particlesArg.found() ? particlesArg.value() : 100;
    std::string outputFile =
        outputArg.found() ? outputArg.value() : "/dev/null";

    // Uncertainty parameters
    double sigX2 = 0.25 * 0.25;
    double sigY2 = 0.1 * 0.1;
    double sigAlpha2 = 0.1 * 0.1;  // Rotation
    double sigBeta2 = 0.01 * 0.01; // Bearing angle
    double sigR2 = 0.08 * 0.08;    // Range

    auto inputLog = readLog(inputFile);

    std::vector<double> initialMeasurement = inputLog.front();
    int numLandmarks = static_cast<int>(initialMeasurement.size() / 2);
    inputLog.erase(inputLog.begin());

    SLAM *slam = nullptr;
    if (strcmp(slamAlgorithm, "EKF") == 0) {
        slam =
            new EKFSLAM(numLandmarks, sigX2, sigY2, sigAlpha2, sigBeta2, sigR2);
    } else if (strcmp(slamAlgorithm, "Fast") == 0) {
        slam = new FastSLAM(numParticles, numLandmarks, sigX2, sigY2, sigAlpha2,
                            sigBeta2, sigR2);
    } else if (strcmp(slamAlgorithm, "GraphBased") == 0) {
        slam = new GraphBasedSLAM(numLandmarks);
    } else {
        panic("Unknown algorithm: %s", slamAlgorithm);
    }

    std::vector<std::vector<double>> outputLog;

    // ROI begins
    zsim_roi_begin();
    
    uint64_t cid = 0;

    if(deadlines){
        cid = hwc_create_contract();
        hwc_add_core(cid);
        hwc_set_deadline(cid, 100);
    }

    for(int i = 0; i < num_runs; i++) {
        // outputLog.push_back(slam->getStatus());
        if(should_m5_exit) m5_exit(0);
        if(deadlines) hwc_start_roi(cid);
        for (auto input : inputLog) {
            // if(!(num_iters % 1000)) {
            //     char m5_buf[32];
            //     int len = sprintf(m5_buf, "%d\n", num_iters);
            //     m5_write_file(m5_buf, len, 0, "slam_stats.txt");
            // }
            if (input.size() == 2) {
                // Control input
                slam->motionUpdate(input[0], input[1], input[2]);
            } else if (input.size() == 2 * static_cast<size_t>(numLandmarks)) {
                // Measurement input
                slam->measurementUpdate(input);
            } else {
                assert(false);
            }

            // outputLog.push_back(slam->getStatus());
        }

        if(deadlines) hwc_end_roi(cid);
        if(should_m5_exit) m5_exit(0);
    }

    if(deadlines) {
        hwc_remove_core(cid);
        hwc_delete_contract(cid);
    }

    zsim_roi_end();
    // ROI ends

    // Write the output log
    // std::ofstream outLogFile;
    // outLogFile.open(outputFile);
    // for (auto l : outputLog) {
    //     for (auto e : l) {
    //         outLogFile << std::setprecision(4) << e << " ";
    //     }
    //     outLogFile << std::endl;
    // }
    // outLogFile.close();
    delete slam;

    return 0;
}