#include <list>
#include <iomanip>
#include <iostream>
#include <fstream>
#include <boost/algorithm/string.hpp>
#include <boost/tokenizer.hpp>
#include <boost/lexical_cast.hpp>

#include "tsp.hpp"

#define print std::cout
#define newl  std::endl
void check()
{
    robot_path graph;
    std::vector<comm_edge_t> in_edges;

    std::string edge_str;
    std::ifstream mapfile;
    mapfile.open("./map.txt");
    while (!mapfile.eof()) {
        std::vector<std::string> vec;
        getline(mapfile, edge_str);
        boost::split(vec, edge_str, boost::is_any_of(" ,/"));
        if (vec.size() != 3) {
            print << "Invalid Line: " << edge_str << newl;
            continue;
        }

        comm_edge_t e(
            vec[0], vec[1],
            boost::lexical_cast<double>(vec[2]));
        e.task(true);
        in_edges.push_back(e);
    }
    mapfile.close();

    std::string u = in_edges.front().source_;
    in_edges = graph.optimal_path(u, u, in_edges);

    for (size_t i = 0; i < in_edges.size(); ++i) {
        if (i % 10 == 0)
            print << newl;
        print << "[" << in_edges[i].source_ << ", " << in_edges[i].target_ << "] ";
    }
}

int main()
{
    check();

    return 0;
}
