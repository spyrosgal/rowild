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
#include "pp.h"
#include "zsim_hooks.h"
#include <fstream>
#include <math.h>
#include <string>
#include <vector>
#include <random>
#include <iostream>

#include "hw_contracts.h"

std::vector<Point> readInputFile(const char *fileName) {
    std::vector<Point> path;

    // std::ifstream pathFile;
    // pathFile.open(fileName);
    // assert(pathFile.good());

    // std::string line;
    // while (std::getline(pathFile, line)) {
    //     double x, y;
    //     std::stringstream ss(line);
    //     ss >> x >> y;
    //     path.push_back(Point(x, y));
    // }

    double lower_bound = 0;
    double upper_bound = 10000;
    std::uniform_real_distribution<double> unif(lower_bound,upper_bound);
    std::default_random_engine re;
    for(int i = 0; i < 100000; i++) {
        if(!(i%10000)) std::cout << i << std::endl;
        path.push_back(Point(unif(re), unif(re)));
    }

    std::cout << "Done creating path" << std::endl;

    // pathFile.close();

    return path;
}

int main(int argc, const char **argv) {
    using args::KVArg;
    using args::Parser;

    Parser parser(argv[0], argc, argv, false);
    KVArg<int> numRunsArg(parser, "num_runs", "", "Number of repetitions");
    KVArg<int> m5exitArg(parser, "m5_exit", "", "Whether m5_exit should be called before and after the benchmark");
    KVArg<int> deadlinesArg(parser, "deadlines", "", "Whether we should actually track deadlines");
    KVArg<std::string> pathFileArg(parser, "path", "", "Input path file");
    KVArg<double> distArg(parser, "dist", "", "Lookahead distance");
    KVArg<std::string> outArg(parser, "output", "", "Output log file");

    if (!parser.parse()) assert(false);

    assert_msg(pathFileArg.found(), "Input file is not provided");

    const int num_runs = numRunsArg.found() ? numRunsArg.value() : 2;
    const int should_m5_exit = m5exitArg.found() ? m5exitArg.value() : 1;
    const int deadlines = deadlinesArg.found() ? deadlinesArg.value() : 1;
    const char *pathFile = pathFileArg.value().c_str();
    double dist = distArg.found() ? distArg.value() : 2.0;
    std::string outputFile = outArg.found() ? outArg.value() : "/dev/null";

    auto path = readInputFile(pathFile);
    Point robotPosition = {1.0, 0.5};

    PurePursuit *tracker = new PurePursuit(path, dist);

    uint64_t cid = 0;

    if(deadlines) {
        cid = hwc_create_contract();
        hwc_add_core(cid);
        hwc_set_deadline(cid, 350);
        hwc_update_linreg(cid, 0, 2.201125e+00);
        hwc_update_linreg(cid, 1, 2.491702e-02);
        hwc_update_linreg(cid, 2, -7003544.366);
    }

    for(int i = 0; i < num_runs; i++) {
        if(should_m5_exit) m5_exit(0);
        if(deadlines) hwc_start_roi(cid);

        // ROI begins
        zsim_roi_begin();

        Point lookAhead = tracker->getLookAheadPoint(robotPosition);

        zsim_roi_end();

        if(deadlines) hwc_end_roi(cid);
        if(should_m5_exit) m5_exit(0);
    }
    // ROI ends

    if(deadlines) {
        hwc_remove_core(cid);
        hwc_delete_contract(cid);
    }

    // Write the output trajectory
    // std::ofstream outLog;
    // outLog.open(outputFile);
    // outLog << lookAhead.x << " " << lookAhead.y << std::endl;
    // outLog.close();

    return 0;
}
