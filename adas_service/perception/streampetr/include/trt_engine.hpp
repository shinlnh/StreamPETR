#ifndef TRT_ENGINE_HPP
#define TRT_ENGINE_HPP

#include <iostream>
#include <fstream>
#include <filesystem>
#include <string>
#include <vector>
#include <memory>
#include <chrono>
#include <unordered_map>

#include <cuda_fp16.h>
#include <cuda_runtime_api.h>
#include <NvInferRuntime.h>

#define LASTERR()                                           \
{                                                           \
    auto code = cudaGetLastError();                         \
    if (code != cudaSuccess)                                \
    {                                                       \
        std::cout << cudaGetErrorString(code) << std::endl; \
    }                                                       \
}

using namespace nvinfer1;

struct TRTDeleter
{
    template <typename T>
    void operator()(T *obj) const {
        if (obj) delete obj;
    }
};
struct CudaGraphDeleter {
    void operator()(cudaGraph_t graph) {
        if (graph) cudaGraphDestroy(graph);
    }
};
struct CudaGraphExecDeleter {
    void operator()(cudaGraphExec_t graph_exec) {
        if (graph_exec) cudaGraphExecDestroy(graph_exec);
    }
};
struct CudaStreamDeleter {
    void operator()(cudaStream_t stream) const {
        if (stream) cudaStreamDestroy(stream);
    }
};

using RuntimePtr = std::unique_ptr<IRuntime, TRTDeleter>;
using CudaStreamPtr = std::unique_ptr<CUstream_st, CudaStreamDeleter>;
using GraphPtr = std::unique_ptr<CUgraph_st, CudaGraphDeleter>;
using GraphExecPtr = std::unique_ptr<CUgraphExec_st, CudaGraphExecDeleter>;

class Logger : public ILogger
{
public:
    void log(Severity severity, const char *msg) noexcept override
    {
        // Only print error messages
        if (severity == Severity::kERROR)
        {
            std::cerr << msg << std::endl;
        }
    }
};

inline unsigned int getElementSize(DataType t)
{
    switch (t)
    {
    case DataType::kINT32:
        return 4;
    case DataType::kFLOAT:
        return 4;
    case DataType::kHALF:
        return 2;
    case DataType::kINT8:
        return 1;
    default:
        break;
    }
    throw std::runtime_error("Invalid DataType.");
    return 0;
}

inline std::string getTypeName(DataType t)
{
    switch (t)
    {
    case DataType::kINT32:
        return "Int32";
    case DataType::kFLOAT:
        return "Float";
    case DataType::kHALF:
        return "Half";
    case DataType::kINT8:
        return "Int8";
    default:
        break;
    }
    throw std::runtime_error("Invalid DataType.");
    return "Invalid DataType";
}

struct Tensor
{
    std::string name;
    void *ptr;
    Dims dim;
    int32_t volume = 1;
    DataType dtype;
    TensorIOMode iomode;

    Tensor(std::string name, Dims dim, DataType dtype) : name(name), dim(dim), dtype(dtype)
    {
        if (dim.nbDims == 0) {
            volume = 0;
        }
        else {
            volume = 1;
            for (int i = 0; i < dim.nbDims; i++) {
                volume *= dim.d[i];
            }
        }
        cudaMalloc(&ptr, volume * getElementSize(dtype));
    }

    int32_t nbytes()
    {
        return volume * getElementSize(dtype);
    }

    int32_t shape(int32_t id)
    {
        if (id < 0) id = dim.nbDims + id;
        if (id < 0 || id >= dim.nbDims) return -1;
        return dim.d[id];
    }

    void copy(std::shared_ptr<Tensor> other, cudaStream_t stream)
    {
        if (this->dtype != other->dtype) {
            throw std::runtime_error(
                "Tensor " + this->name +
                " expected " + getTypeName(this->dtype) +
                " but got " + getTypeName(other->dtype) + " buffer!"
            );
        }
        // copy from 'other'
        cudaMemcpyAsync(ptr, other->ptr, nbytes(), cudaMemcpyDeviceToDevice, stream);
    }

    void copy(const std::vector<float> &data, cudaStream_t stream)
    {
        if (this->dtype != DataType::kFLOAT) {
            throw std::runtime_error(
                "Tensor " + this->name +
                " expected " + getTypeName(this->dtype) + " type but got float buffer!"
            );
        }
        // copy from data buffer
        cudaMemcpyAsync(ptr, (void*)data.data(), nbytes(), cudaMemcpyHostToDevice, stream);
    }

    void copy(const float* const data, cudaStream_t stream)
    {
        if (this->dtype != DataType::kFLOAT) {
            throw std::runtime_error(
                "Tensor " + this->name +
                " expected " + getTypeName(this->dtype) + " type but got float buffer!"
            );
        }
        // copy from data buffer
        cudaMemcpyAsync(ptr, (void*)data, nbytes(), cudaMemcpyHostToDevice, stream);
    }

    template <class Htype = float, class Dtype = float>
    void load(std::string fname)
    {
        size_t hsize = volume * sizeof(Htype);
        size_t dsize = volume * getElementSize(dtype);
        std::vector<char> b1(hsize);
        std::vector<char> b2(dsize);
        std::ifstream file_(fname, std::ios::binary);
        if (file_.fail())
        {
            std::cerr << fname << " missing!" << std::endl;
            return;
        }
        file_.read(b1.data(), hsize);
        file_.close();
        Htype *hbuffer = reinterpret_cast<Htype *>(b1.data());
        Dtype *dbuffer = reinterpret_cast<Dtype *>(b2.data());
        // in some cases we want to load from different dtype
        for (int i = 0; i < volume; i++)
        {
            dbuffer[i] = (Dtype)hbuffer[i];
        }

        cudaMemcpy(ptr, b2.data(), dsize, cudaMemcpyHostToDevice);
    }

    template <class Htype = float, class Dtype = float>
    void save(std::string fname)
    {
        size_t hsize = volume * sizeof(Htype);
        size_t dsize = volume * getElementSize(dtype);
        std::vector<char> b1(hsize);
        std::vector<char> b2(dsize);
        std::ofstream file_(fname, std::ios::binary);
        if (file_.fail())
        {
            std::cerr << fname << " can't open!" << std::endl;
            return;
        }
        Htype *hbuffer = reinterpret_cast<Htype *>(b1.data());
        Dtype *dbuffer = reinterpret_cast<Dtype *>(b2.data());
        cudaMemcpy(b2.data(), ptr, dsize, cudaMemcpyDeviceToHost);
        // in some cases we want to save to different dtype
        for (int i = 0; i < volume; i++)
        {
            hbuffer[i] = (Htype)dbuffer[i];
        }
        file_.write(b2.data(), hsize);
        file_.close();
    }

    std::vector<float> cpu()
    {
        std::vector<float> buffer(volume);
        cudaMemcpy(buffer.data(), ptr, volume * sizeof(float), cudaMemcpyDeviceToHost);
        return buffer;
    }

    std::vector<char> load_ref(std::string fname)
    {
        size_t bsize = volume * sizeof(float);
        std::vector<char> buffer(bsize);
        std::ifstream file_(fname, std::ios::binary);
        file_.read(buffer.data(), bsize);
        return buffer;
    }
}; // struct Tensor

// std::ostream &operator<<(std::ostream &os, Tensor &t)
// {
//     os << "[" << (int)(t.iomode) << "] ";
//     os << t.name << ", [";

//     for (int nd = 0; nd < t.dim.nbDims; nd++)
//     {
//         if (nd == 0)
//         {
//             os << t.dim.d[nd];
//         }
//         else
//         {
//             os << ", " << t.dim.d[nd];
//         }
//     }
//     std::cout << "]";
//     std::cout << ", type = " << int(t.dtype);
//     return os;
// }

class SubNetwork
{
    std::unique_ptr<ICudaEngine> engine;
    std::unique_ptr<IExecutionContext> context;

public:
    std::unordered_map<std::string, std::shared_ptr<Tensor>> bindings;
    bool use_cuda_graph = false;
    GraphPtr graph;
    GraphExecPtr graph_exec;

    SubNetwork(std::string engine_path, IRuntime *runtime)
    {
        std::ifstream engine_file(engine_path, std::ios::binary);
        if (!engine_file)
        {
            throw std::runtime_error("Error opening engine file: " + engine_path);
        }
        engine_file.seekg(0, engine_file.end);
        long int fsize = engine_file.tellg();
        engine_file.seekg(0, engine_file.beg);

        // Read the engine file into a buffer
        std::vector<char> engineData(fsize);

        engine_file.read(engineData.data(), fsize);
        engine.reset(runtime->deserializeCudaEngine(engineData.data(), fsize));
        context.reset(engine->createExecutionContext());

        int nb = engine->getNbIOTensors();

        for (int n = 0; n < nb; n++)
        {
            std::string name = engine->getIOTensorName(n);
            Dims d = engine->getTensorShape(name.c_str());
            DataType dtype = engine->getTensorDataType(name.c_str());
            bindings[name] = std::make_shared<Tensor>(name, d, dtype);
            bindings[name]->iomode = engine->getTensorIOMode(name.c_str());
            // std::cout << *(bindings[name]) << std::endl;
            context->setTensorAddress(name.c_str(), bindings[name]->ptr);
        }
    }

    void Enqueue(cudaStream_t stream)
    {
        if (this->use_cuda_graph)
        {
            cudaGraphLaunch(graph_exec.get(), stream);
        }
        else
        {
            context->enqueueV3(stream);
        }
    }

    ~SubNetwork()
    {
    }

    void EnableCudaGraph(cudaStream_t stream)
    {
        cudaGraph_t rawGraph;
        cudaGraphExec_t rawGraphExec;
        // run first time to avoid allocation
        this->Enqueue(stream);
        cudaStreamSynchronize(stream);

        cudaStreamBeginCapture(stream, cudaStreamCaptureModeGlobal);
        this->Enqueue(stream);
        cudaStreamEndCapture(stream, &rawGraph);
        this->use_cuda_graph = true;
#if CUDART_VERSION < 12000
        cudaGraphInstantiate(&rawGraphExec, rawGraph, NULL, NULL, 0);
#else
        cudaGraphInstantiate(&rawGraphExec, rawGraph, 0);
#endif
        this->graph.reset(rawGraph);
        this->graph_exec.reset(rawGraphExec);
    }
}; // class SubNetwork

class Duration
{
private:
    // stat
    std::vector<float> stats;
    cudaEvent_t b, e;
    std::string m_name;

public:
    Duration(std::string name) : m_name(name)
    {
        cudaEventCreate(&b);
        cudaEventCreate(&e);
    }

    void MarkBegin(cudaStream_t s)
    {
        cudaEventRecord(b, s);
    }

    void MarkEnd(cudaStream_t s)
    {
        cudaEventRecord(e, s);
    }

    float Elapsed()
    {
        float val;
        cudaEventElapsedTime(&val, b, e);
        stats.push_back(val);
        return val;
    }
}; // class Duration

#endif // TRT_ENGINE_HPP