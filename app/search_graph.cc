#include <iostream>
#include <cmath>

#include "graph_search.h"

int main(int argc, char** argv) {
    if (argc < 7) {
        std::cerr << "Usage: " << argv[0] << " file_to_read initial_p initial_eps tightening_rate method file_to_write" << std::endl;
        exit(1);
    }

    // parse input
    String file_to_read = argv[1];
    const RealNum initial_p = std::stof(argv[2]);
    const RealNum initial_eps = std::stof(argv[3]);
    RealNum tightening_rate = std::stof(argv[4]);
    // 0 -- no lazy
    // 1 -- lazy A*
    // 2 -- complete lazy
    Idx method = std::stoi(argv[5]);
    String file_to_write = argv[6];
    

    Inspection::GPtr graph(new Inspection::Graph);
    graph->ReadFromFiles(file_to_read, false);
    // graph->ReadFromFiles(file_to_read, true, 5);

    GraphSearch search(graph);

    RealNum p = initial_p;
    RealNum eps = initial_eps;
    SizeType step = 1;
    SizeType addtional = 0;

    std::ofstream fout;
    fout.open(file_to_write);
    if (!fout.is_open()) {
        std::cerr << file_to_write << " cannot be opened!" << std::endl;
        exit(1);
    }

    std::ofstream fout_result;
    fout_result.open(file_to_write + "_result");

    if (!fout_result.is_open()) {
        std::cerr << file_to_write + "_result" << " cannot be opened!" << std::endl;
        exit(1);
    }

    search.PrintTitle(std::cout);
    std::vector<Idx> path;
    for (SizeType graph_size = step; graph_size <= graph->NumVertices(); graph_size += step) {
        search.ExpandVirtualGraph(graph_size, method > 0);
        addtional += step;

        auto update_rate = tightening_rate;
        //auto update_rate = pow(tightening_rate, sqrt(graph_size));
        p += (1 - p)*update_rate;
        eps += (0 - eps)*update_rate;

        if (search.ResultCoverageSize() / (RealNum)search.VirtualGraphCoverageSize() > p
            && addtional < 500) {
            continue;
        }

        if (method == 1) {
            path = search.SearchVirtualGraph(p, eps);
        }
        else {
            path = search.SearchVirtualGraphCompleteLazy(p, eps);
        }
        
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
