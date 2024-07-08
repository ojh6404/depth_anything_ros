#!/usr/bin/env sh

if [ $# -eq 0 ]; then
    echo "Usage: $0 <onnx_dir>"
    exit 1
fi

onnx_dir="$1"

if [ ! -d "$onnx_dir" ]; then
    echo "Directory $onnx_dir does not exist."
    exit 1
fi

for onnx_file in "$onnx_dir"/*.onnx
do
    if [ -f "$onnx_file" ]; then
        base_name=$(basename "${onnx_file%.onnx}")
        trtexec --onnx="$onnx_file" --saveEngine="$onnx_dir/${base_name}.engine"
        echo "Exported $onnx_file to $onnx_dir/${base_name}.engine"
    fi
done
echo "Exported all ONNX files to TensorRT engines."
