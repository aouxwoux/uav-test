#include "demo_data.hpp"
#include <iostream>
int main(int argc, char** argv) {
    if (argc != 2) { std::cerr << "Usage: make_demo OUTPUT_FOLDER\n"; return 2; }
    try {
        demo::generate(argv[1]);
        return 0;
    } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
