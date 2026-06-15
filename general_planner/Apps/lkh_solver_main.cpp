#include <cstdlib>
#include <iostream>

int solveTSPLKH(const char *input_file);

int main(int argc, char **argv) {
    if (argc < 2 || argv[1] == nullptr) {
        std::cerr << "Usage: general_planner_lkh <parameter_file>\n";
        return EXIT_FAILURE;
    }

    return solveTSPLKH(argv[1]);
}
