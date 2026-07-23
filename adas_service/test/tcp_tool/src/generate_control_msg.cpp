#include <iostream>
#include <fstream>
#include "../include/json.hpp"

using json = nlohmann::json;

int main() {
    // Read the JSON file
    std::ifstream input("configs/config.json");
    json j;
    input >> j;

    // Generate the control_msgs.h file
    std::ofstream output("include/control_msg.h");
    output << "#pragma once\n\n";
    output << "#include <cstdint>\n\n";
    
    output << "typedef struct {\n";

    for (auto it = j.begin(); it != j.end(); ++it) {
        const std::string& variableName = it.key();
        const std::string& dataType = it.value();

        // Generate the member variable declaration
        output << "    " << dataType << " " << variableName << ";\n";
    }

    output << "} control_msg;\n\n";

    output.close();
    return 0;
}
