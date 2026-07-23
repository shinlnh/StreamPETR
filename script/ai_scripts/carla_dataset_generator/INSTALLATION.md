# Installation

Outside of the requirements in the requirements.txt file. You need to install additional toolkits in order to interact with the GPU.
Here is the list of what you need:
* CUDA 11.1
* CuDNN 9
* TensorRT 8.4.5.1

## CUDA installation
CUDA local installation method taken directly from the guide [here](https://developer.nvidia.com/cuda-11.1.0-download-archive?target_os=Linux&target_arch=x86_64&target_distro=Ubuntu&target_version=2004&target_type=deblocal):
```
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/cuda-ubuntu2004.pin
sudo mv cuda-ubuntu2004.pin /etc/apt/preferences.d/cuda-repository-pin-600
wget https://developer.download.nvidia.com/compute/cuda/11.1.0/local_installers/cuda-repo-ubuntu2004-11-1-local_11.1.0-455.23.05-1_amd64.deb
sudo dpkg -i cuda-repo-ubuntu2004-11-1-local_11.1.0-455.23.05-1_amd64.deb
sudo apt-key add /var/cuda-repo-ubuntu2004-11-1-local/7fa2af80.pub
sudo apt-get update
sudo apt-get -y install cuda-11-1 cuda-drivers
```
## CuDNN installation
Both CuDNN and TensorRT requires extensive copying which can't seem to be automated. You might see links online that installs using apt-get but I can't seem to get it to work. I managed to get it working using the tarball installation. Here are the steps:


Download the CuDNN libraries:
```
wget https://developer.download.nvidia.com/compute/cudnn/redist/cudnn/linux-x86_64/cudnn-linux-x86_64-9.0.0.312_cuda11-archive.tar.xz
```

Extract the tarball:
```
xzvf cudnn-linux-x86_64-9.0.0.312_cuda11-archive.tar.xz 
```
The extracted directory should look like this:
```
├── include
│   ├── ...
├── lib
│   ├── ...
│ 
└── LICENSE
```
We now need to do some additional copying
```
cd cudnn-linux-x86_64-9.0.0.312_cuda11-archive/
sudo cp -P include/cudnn.h /usr/local/cuda-11.1/include/
sudo cp -P lib/libcudnn* /usr/local/cuda-11.1/lib64
sudo chmod a+r /usr/local/cuda-11.1/lib64/libcudnn*
```

## TensorRT installation
TensorRT also requires install from the tarball to get the version we need (8.4.5.1). Here are the steps:
Go [here](https://developer.nvidia.com/nvidia-tensorrt-8x-download) and downloads the tarball. I selected *TensorRT 8.4 EA for Ubuntu 20.04 and CUDA 11.6 DEB local repo Package*.
Extract the tarball:
```
tar -xzvf TensorRT-8.4.1.5.Linux.x86_64-gnu.cuda-11.6.cudnn8.4.tar.gz 
```
The extracted directory should look like this:
```
bin  data  doc  include  lib  onnx_graphsurgeon  python  samples  targets
```

Add the absolute path to the TensorRT lib directory to the environment variable LD_LIBRARY_PATH:
```
export LD_LIBRARY_PATH=<TensorRT-${version}/lib>:$LD_LIBRARY_PATH
```
Install the Python TensorRT wheel file (replace cp3x with the desired Python version, for example, cp310 for Python 3.10).
```
cd TensorRT-${version}/python
python3 -m pip install tensorrt-*-cp3x-none-linux_x86_64.whl
```
Install the Python onnx-graphsurgeon wheel file.
```
cd TensorRT-${version}/onnx_graphsurgeon
python3 -m pip install onnx_graphsurgeon-0.5.0-py2.py3-none-any.whl
```

Note the LD_LIBRARY_PATH must be added in every new shell. You will need to add the bin directory to PATH too if you wish to use trtexec. Here's the script, you can add it to your bashrc if you want.
```
TENSORRT_BASE="/prj/es-automotive/adas/common/TensorRT-8.4.1.5"
export PATH="${TENSORRT_BASE}/bin:${PATH}"
export LD_LIBRARY_PATH="${TENSORRT_BASE}/lib:${LD_LIBRARY_PATH}"
``` 