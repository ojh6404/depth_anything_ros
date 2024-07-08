#!/usr/bin/bash

git submodule update --init --recursive
pip3 install -r requirements.txt
./scripts/onnx2tensorrt.sh trained_data
