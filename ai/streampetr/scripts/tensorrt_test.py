import tensorrt as trt
import pycuda.driver as cuda
import pycuda.autoinit
import numpy as np
import time
import os

class TensorRTBenchmark:
    def __init__(self, onnx_path, engine_path=None, precision="fp32"):
        self.trt_logger = trt.Logger(trt.Logger.WARNING)
        self.onnx_path = onnx_path
        self.engine_path = engine_path
        self.precision = precision
        self.engine = None
        self.context = None
        self.inputs = []
        self.outputs = []
        self.allocations = []
        self.stream = cuda.Stream()

        # Check hardware support
        builder = trt.Builder(self.trt_logger)
        if self.precision == "int8" and not builder.platform_has_fast_int8:
            self.precision = "fp16"
            print("INT8 not supported on this GPU, falling back to FP16.")
        if self.precision == "fp16" and not builder.platform_has_fast_fp16:
            self.precision = "fp32"
            print("FP16 not supported on this GPU, falling back to FP32.")

        # create default engine path if none is given
        if self.engine_path is None:
            self.engine_path = self.onnx_path + "_" + self.precision + ".engine"

    def _layernorm_force_fp32(self, network):
        # Find Pow node (indicate the start of LayerNorm)
        for i in range(network.num_layers):
            pow_layer = network.get_layer(i)
            if pow_layer.type != trt.LayerType.ELEMENTWISE:
                continue

            pow_layer.__class__ = trt.IElementWiseLayer
            if pow_layer.op != trt.ElementWiseOperation.POW:
                continue

            # Find downstream reduce layer
            reduce_layer = None
            for j in range(network.num_layers):
                pow_output = pow_layer.get_output(0)
                next_layer = network.get_layer(j)
                next_layer_inputs = [next_layer.get_input(id) for id in range(next_layer.num_inputs)]

                # check if use pow output
                if not pow_output in next_layer_inputs:
                    continue

                if next_layer.type != trt.LayerType.REDUCE:
                    continue

                reduce_layer = next_layer
                break

            if reduce_layer is None:
                continue

            # Find downstream add layer
            add_layer = None
            for j in range(network.num_layers):
                reduce_output = reduce_layer.get_output(0)
                next_layer = network.get_layer(j)
                next_layer_inputs = [next_layer.get_input(id) for id in range(next_layer.num_inputs)]

                # check if use reduce output
                if not reduce_output in next_layer_inputs:
                    continue

                if next_layer.type != trt.LayerType.ELEMENTWISE:
                    continue

                next_layer.__class__ = trt.IElementWiseLayer
                if next_layer.op != trt.ElementWiseOperation.SUM:
                    continue

                add_layer = next_layer
                break

            if add_layer is None:
                continue

            # Find downstream sqrt layer
            sqrt_layer = None
            for j in range(network.num_layers):
                add_output = add_layer.get_output(0)
                next_layer = network.get_layer(j)
                next_layer_inputs = [next_layer.get_input(id) for id in range(next_layer.num_inputs)]

                # check if use add output
                if not add_output in next_layer_inputs:
                    continue

                if next_layer.type != trt.LayerType.UNARY:
                    continue

                next_layer.__class__ = trt.IUnaryLayer
                if next_layer.op != trt.UnaryOperation.SQRT:
                    continue

                sqrt_layer = next_layer
                break

            if sqrt_layer is None:
                continue

            # Find downstream div layer
            div_layer = None
            for j in range(network.num_layers):
                sqrt_output = sqrt_layer.get_output(0)
                next_layer = network.get_layer(j)
                next_layer_inputs = [next_layer.get_input(id) for id in range(next_layer.num_inputs)]

                # check if use sqrt output
                if not sqrt_output in next_layer_inputs:
                    continue

                if next_layer.type != trt.LayerType.ELEMENTWISE:
                    continue

                next_layer.__class__ = trt.IElementWiseLayer
                if next_layer.op != trt.ElementWiseOperation.DIV:
                    continue

                div_layer = next_layer
                break

            if div_layer is None:
                continue
            
            # Force the layer to compute and output in FP32
            for layer in [pow_layer, reduce_layer, add_layer, sqrt_layer, div_layer]:
                layer.precision = trt.DataType.FLOAT
                layer.set_output_type(0, trt.DataType.FLOAT)

            print(f"Forced {pow_layer.name} and subsequences to FP32")

    def build_engine(self):
        """Builds a TensorRT engine from an ONNX file."""
        if os.path.exists(self.engine_path):
            print(f"Loading existing engine from {self.engine_path}...")
            with open(self.engine_path, "rb") as f, trt.Runtime(self.trt_logger) as runtime:
                self.engine = runtime.deserialize_cuda_engine(f.read())
        else:
            print(f"Building engine from {self.onnx_path}...")
            builder = trt.Builder(self.trt_logger)
            network = builder.create_network(1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH))
            parser = trt.OnnxParser(network, self.trt_logger)
            
            with open(self.onnx_path, 'rb') as model:
                if not parser.parse(model.read()):
                    for error in range(parser.num_errors):
                        print(parser.get_error(error))
                    return False

            config = builder.create_builder_config()
            # Set workspace limit (e.g., 1GB)
            config.set_memory_pool_limit(trt.MemoryPoolType.WORKSPACE, 2 << 30)

            if self.precision == "int8":
                if builder.platform_has_fast_int8:
                    config.set_flag(trt.BuilderFlag.INT8)

                    # After parsing the ONNX model into 'network'
                    for i in range(network.num_layers):
                        layer = network.get_layer(i)
                        for j in range(layer.num_outputs):
                            tensor = layer.get_output(j)
                            # Set a dummy dynamic range to enable INT8 kernels
                            tensor.dynamic_range = (-127.0, 127.0)

                    # Also set for network inputs
                    for i in range(network.num_inputs):
                        network.get_input(i).dynamic_range = (-127.0, 127.0)

                    # Force FP32 for LayerNorm layers to avoid losing accuracy
                    self._layernorm_force_fp32(network)
                    config.set_flag(trt.BuilderFlag.OBEY_PRECISION_CONSTRAINTS)
                else:
                    self.precision = "fp16"
                    print("INT8 not supported on this GPU, falling back to FP16.")

                if builder.platform_has_fast_fp16:
                    config.set_flag(trt.BuilderFlag.FP16)

            if self.precision == "fp16":
                if builder.platform_has_fast_fp16:
                    config.set_flag(trt.BuilderFlag.FP16)

                    # Force FP32 for LayerNorm layers to avoid losing accuracy
                    self._layernorm_force_fp32(network)
                    config.set_flag(trt.BuilderFlag.OBEY_PRECISION_CONSTRAINTS)
                else:
                    self.precision = "fp32"
                    print("FP16 not supported on this GPU, falling back to FP32.")

            serialized_engine = builder.build_serialized_network(network, config)
            with open(self.engine_path, "wb") as f:
                f.write(serialized_engine)
            
            runtime = trt.Runtime(self.trt_logger)
            self.engine = runtime.deserialize_cuda_engine(serialized_engine)

        self.context = self.engine.create_execution_context()
        self._allocate_buffers()
        return True

    def _allocate_buffers(self):
        """Allocates memory on GPU for inputs and outputs."""
        for i in range(self.engine.num_io_tensors):
            name = self.engine.get_tensor_name(i)
            dtype = trt.nptype(self.engine.get_tensor_dtype(name))
            shape = self.engine.get_tensor_shape(name)
            size = trt.volume(shape)
            
            # Host (CPU) and Device (GPU) memory
            host_mem = cuda.pagelocked_empty(size, dtype)
            device_mem = cuda.mem_alloc(host_mem.nbytes)
            
            self.allocations.append(int(device_mem))
            if self.engine.get_tensor_mode(name) == trt.TensorIOMode.INPUT:
                self.inputs.append({'host': host_mem, 'device': device_mem, 'name': name})
            else:
                self.outputs.append({'host': host_mem, 'device': device_mem, 'name': name})

    def run_benchmark(self, iterations=100, warmup=10):
        print(f"Starting benchmark for {iterations} iterations...")
        
        # 1. Prepare random input
        for inp in self.inputs:
            np.copyto(inp['host'], np.random.random(inp['host'].shape).astype(inp['host'].dtype))

        # 2. Warmup
        for _ in range(warmup):
            self.infer()

        # 3. Timed Loop
        cuda.Context.synchronize()
        start_time = time.time()
        for _ in range(iterations):
            self.infer()
        cuda.Context.synchronize()
        end_time = time.time()

        total_time = end_time - start_time
        avg_latency = (total_time / iterations) * 1000
        fps = iterations / total_time
        
        print("-" * 30)
        print(f"Average Latency: {avg_latency:.3f} ms")
        print(f"Average FPS: {fps:.2f}")
        print("-" * 30)

    def infer(self):
        """Single inference step."""
        # Host to Device
        for inp in self.inputs:
            cuda.memcpy_htod_async(inp['device'], inp['host'], self.stream)
        
        # Set tensor addresses
        for i, addr in enumerate(self.allocations):
            self.context.set_tensor_address(self.engine.get_tensor_name(i), addr)
            
        # Execute
        self.context.execute_async_v3(self.stream.handle)
        
        # Device to Host
        for out in self.outputs:
            cuda.memcpy_dtoh_async(out['host'], out['device'], self.stream)
        
        self.stream.synchronize()

if __name__ == "__main__":
    # # Update these paths to your files
    # ONNX_MODEL_PATH = "simplify_extract_img_feat.onnx"
    
    # bench = TensorRTBenchmark(ONNX_MODEL_PATH, precision="fp16") # or "fp32"
    # if bench.build_engine():
    #     bench.run_benchmark(iterations=100)

    ONNX_MODEL_PATH = "simplify_pts_head_memory.onnx"
    bench = TensorRTBenchmark(ONNX_MODEL_PATH, precision="fp16") # or "fp32"
    if bench.build_engine():
        bench.run_benchmark(iterations=100)

    # ONNX_MODEL_PATH = "simplify_monolith.onnx"
    # bench = TensorRTBenchmark(ONNX_MODEL_PATH, precision="fp16") # or "fp32"
    # if bench.build_engine():
    #     bench.run_benchmark(iterations=100)