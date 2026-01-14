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
#include "sdcar.h"
#include "zsim_hooks.h"
#include <fstream>
#include <string>

// Serialize + read TSC
static inline uint64_t rdtsc() {
    unsigned int lo, hi;
    __asm__ volatile (
        "cpuid\n\t"        // serialize
        "rdtsc\n\t"        // read TSC into EDX:EAX
        : "=a" (lo), "=d" (hi)
        : "a"(0)
        : "%ebx", "%ecx");
    return ((uint64_t)hi << 32) | lo;
}

// Same but for end (use RDTSCP, which is ordered)
static inline uint64_t rdtscp() {
    unsigned int lo, hi;
    __asm__ volatile (
        "rdtscp\n\t"       // read TSC into EDX:EAX, ECX=core ID
        "mov %%eax, %0\n\t"
        "mov %%edx, %1\n\t"
        "cpuid\n\t"        // serialize
        : "=r"(lo), "=r"(hi)
        :
        : "%rax", "%rbx", "%rcx", "%rdx");
    return ((uint64_t)hi << 32) | lo;
}

int main(int argc, const char **argv) {
    using args::FlagArg;
    using args::KVArg;
    using args::Parser;

    // //////////
    // cpu_set_t mask;
    // CPU_ZERO(&mask);
    // int ncpus = 8;
    // for(int i = 20 - ncpus; i < 20; i++)
    //     CPU_SET(i, &mask);  // add core 0
    // // CPU_SET(2, &mask);  // add core 2

    // pid_t pid = 0; // 0 means current process
    // if (sched_setaffinity(pid, sizeof(mask), &mask) != 0) {
    //     perror("sched_setaffinity");
    //     return 1;
    // }

    // // Verify
    // CPU_ZERO(&mask);
    // if (sched_getaffinity(0, sizeof(mask), &mask) == 0) {
    //     printf("Allowed CPUs:");
    //     for (int i = 0; i < CPU_SETSIZE; ++i)
    //         if (CPU_ISSET(i, &mask)) printf(" %d", i);
    //     printf("\n");
    // }
    // //////////

    Parser parser(argv[0], argc, argv, false);
    KVArg<std::string> inputMapArg(parser, "map", "", "Input map file");
    KVArg<double> heuristicWeightArg(parser, "weight", "",
                                     "Heuristic weight of A*");
    KVArg<std::string> aStarHeuristicArg(parser, "heuristic", "",
                                         "A* heuristic");
    FlagArg deepeningArg(parser, "deepening", "", "Enable IDA*");
    KVArg<int> scaleMapArg(parser, "scale-map", "", "Map scale factor");
    KVArg<int> scaleRobotArg(parser, "scale-robot", "", "Robot scale factor");
    KVArg<uint64_t> maxIterationsArg(parser, "max-iters", "",
                                     "Maximum iterations");
    KVArg<std::string> outputPathArg(parser, "output", "", "Output path file");

    if (!parser.parse()) assert(false);

    assert_msg(inputMapArg.found(), "Input map file is not provided");

    std::string inputFile = inputMapArg.value();
    double heuristicWeight =
        heuristicWeightArg.found() ? heuristicWeightArg.value() : 1.0;
    std::string aStarHeuristicStr =
        aStarHeuristicArg.found() ? aStarHeuristicArg.value() : "Euclidean";
    bool idastar = deepeningArg.found();
    int scaleMap = scaleMapArg.found() ? scaleMapArg.value() : 1;
    int scaleRobot = scaleRobotArg.found() ? scaleRobotArg.value() : 1;
    uint64_t maxIterations =
        maxIterationsArg.found() ? maxIterationsArg.value() : 10'000'000;
    std::string outputFile =
        outputPathArg.found() ? outputPathArg.value() : "/dev/null";

    STATE startPos = {0 /*X*/, 0 /*Y*/, 0 /*Theta*/};
    STATE goalPos = {965 /*X*/, 980 /*Y*/, 0 /*Theta*/};

    int robotLength = 10, robotWidth = 4; // Numbers in resolution unit

    // Scaling the robot size
    robotLength *= scaleRobot;
    robotWidth *= scaleRobot;

    SelfDrivingCar sdCar(inputFile, scaleMap, startPos, goalPos, robotLength,
                         robotWidth, aStarHeuristicStr, heuristicWeight,
                         idastar);

    // ROI begins
    // uint64_t start, end;
    // start = rdtsc();

    zsim_roi_begin();

    auto path = sdCar.run(maxIterations);

    zsim_roi_end();
    // end = rdtscp();
    // printf("Cycles: %llu\n", (unsigned long long)(end - start));

    // ROI ends

    // Write the output path
    std::ofstream pathFile;
    pathFile.open(outputFile);
    for (auto dir : path) {
        pathFile << dir << std::endl;
    }
    pathFile.close();

    return 0;
}
