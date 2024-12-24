#include <fstream>
#include <iostream>
#include <memory>
#include "src/reeds_shepp_path.h"
#include "src/basic_type.h"
#include "src/hybrid_a_star.h"
#include "result_plot/cpp_plot.h"

namespace plt = matplotlibcpp;

void printHybridAStarResult(const HybridAStartResult& result, const std::string& file_name) {
    std::ofstream file(file_name);
    if (file.is_open()) {
        for (size_t i = 0; i < result.x.size(); ++i) {
            file << result.x[i] << " " << result.y[i] << " " << result.phi[i] << " " << result.v[i] << std::endl;
        }
        file.close();
        std::cout << "Successfully printed HybridAStarResult to " << file_name << std::endl;
    } else {
        std::cout << "Error opening file " << file_name << std::endl;
    }
}