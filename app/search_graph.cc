// BSD 3-Clause License

// Copyright (c) 2019, The University of North Carolina at Chapel Hill
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:

// 1. Redistributions of source code must retain the above copyright notice, this
//    list of conditions and the following disclaimer.

// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.

// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

//! @author Mengyu Fu

#include <iostream>
#include <cmath>
#include <ctime>

#include "graph_search.h"

int main(int argc, char** argv) {
    if (argc < 9) {
        std::cerr << "Usage: " << argv[0] <<
                  " file_to_read initial_p initial_eps tightening_rate laziness_mode successor_mode batching_ratio file_to_write"
                  << std::endl;
        exit(1);
    }

#if USE_NODE_REUSE
    std::cout << "Node reuse enabled!" << std::endl;
#endif

    // parse input
    String file_to_read = argv[1];
    const RealNum initial_p = std::stof(argv[2]);
    const RealNum initial_eps = std::stof(argv[3]);
    RealNum tightening_rate = std::stof(argv[4]);
    Idx laziness_mode = std::stoi(argv[5]);
    Idx successor_mode = std::stoi(argv[6]);
    RealNum ratio = std::stof(argv[7]);
    std::cout << "ratio:" << ratio << std::endl;
    String file_to_write = argv[8];

    Inspection::GPtr graph(new Inspection::Graph);
    graph->ReadFromFiles(file_to_read, true, false);
    // graph->ReadFromFiles(file_to_read);

    GraphSearch search(graph);

    RealNum p = initial_p;
    RealNum eps = initial_eps;
    SizeType step = 1;
    SizeType addtional = 0;
    std::ofstream fout;
    fout.open(file_to_write);

    std::ofstream fout_result;
    fout_result.open(file_to_write + "_result");

    if (!fout.is_open()) {
        std::cerr << file_to_write << " cannot be opened!" << std::endl;
        exit(1);
    }

    if (!fout_result.is_open()) {
        std::cerr << file_to_write + "_result" << " cannot be opened!" << std::endl;
        exit(1);
    }

    // Lazyiness:
    // 0 -- No lazy
    // 1 -- Lazy SP
    // 2 -- Lazy A* modified (validate when subsuming for the first time)
    // 3 -- Lazy A* (validate only when popped from OPEN list, performance worse than 2, keep for reference)
    search.SetLazinessMode(laziness_mode);

    // Successor mode:
    // 0 -- Direct neighboring successors on the roadmap (default)
    // 1 -- First neighbor that increases inspection coverage
    // 2 -- first neighbor that increases inspection coverage and
    //      there's no other node increasing the coverage along the shortest path from its parent
    search.SetSuccessorMode(successor_mode);

    search.SetSourceIndex(0);
    search.PrintTitle(std::cout);

    std::vector<Idx> path;

    for (SizeType graph_size = step; graph_size <= graph->NumVertices(); graph_size += step) {
        search.ExpandVirtualGraph(graph_size);
        addtional += step;

        auto update_rate = tightening_rate;
        //auto update_rate = pow(tightening_rate, sqrt(graph_size));

        for (auto i = 0; i < step; ++i) {
            p += (1 - p)*update_rate;
            eps += (0 - eps)*update_rate;
        }

        if (search.ResultCoverageSize() / (RealNum)search.VirtualGraphCoverageSize() >= ratio * p
                && addtional < 200) {
            continue;
        }

        search.UpdateApproximationParameters(eps, p);

        std::cout << "Graph size: " << graph_size << std::flush;
        path = search.SearchVirtualGraph();
        std::cout << "\r                                 " << std::flush;

        auto current = std::chrono::system_clock::now();
        std::time_t current_time = std::chrono::system_clock::to_time_t(current);
        std::cout << "\r" << std::ctime(&current_time);

        search.PrintResult(std::cout);
        search.PrintResult(fout);

        fout_result << graph_size << ": ";

        for (auto& p : path) {
            fout_result << p << " ";
        }

        fout_result << std::endl;

        addtional = 0;
    }

    return 0;
}
