#include "model.h"
#include "config.h"
#include "logging.h"
#include <fstream>
#include <iostream>
#include <cassert>

using namespace nvinfer1;

static Logger gLogger;

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <input.wts> <output.engine>" << std::endl;
        return 1;
    }

    std::string wts_name = argv[1];
    std::string engine_name = argv[2];

    std::ifstream wts_file(wts_name);
    if (!wts_file.good()) {
        std::cerr << "ERROR: Weight file not found: " << wts_name << std::endl;
        return 1;
    }
    wts_file.close();

    std::cout << "Building YOLOv5 segmentation engine..." << std::endl;
    std::cout << "  Input weights: " << wts_name << std::endl;
    std::cout << "  Output engine: " << engine_name << std::endl;

    IBuilder* builder = createInferBuilder(gLogger);
    IBuilderConfig* config = builder->createBuilderConfig();

    float gd = 0.33f, gw = 0.50f;
    ICudaEngine* engine = build_seg_engine(OD_BATCH_SIZE, builder, config, DataType::kFLOAT, gd, gw, wts_name);
    assert(engine != nullptr);

    IHostMemory* serialized_engine = engine->serialize();
    assert(serialized_engine != nullptr);

    std::ofstream p(engine_name, std::ios::binary);
    if (!p) {
        std::cerr << "ERROR: Could not open output file: " << engine_name << std::endl;
        return 1;
    }
    p.write(reinterpret_cast<const char*>(serialized_engine->data()), serialized_engine->size());
    p.close();

    std::cout << "Engine saved to " << engine_name << " (" << serialized_engine->size() << " bytes)" << std::endl;

    delete serialized_engine;
    delete engine;
    delete config;
    delete builder;

    return 0;
}
