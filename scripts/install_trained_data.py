#!/usr/bin/env python

import argparse
import multiprocessing

import jsk_data


def download_data(*args, **kwargs):
    p = multiprocessing.Process(target=jsk_data.download_data, args=args, kwargs=kwargs)
    p.start()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-v", "--verbose", dest="quiet", action="store_false")
    args = parser.parse_args()
    args.quiet

    PKG = "depth_anything_ros"

    # depth anything v2 metric hypersim onnx
    download_data(
        pkg_name=PKG,
        path="trained_data/depth_anything_v2_metric_hypersim_vits.onnx",
        url="https://drive.google.com/uc?id=1iEuz-rt5_KryGwBR4D2NfRjRHb6w2Tjj",
        md5="9aeb5b973ee10afb5f7fc6be9163f796",
    )
    download_data(
        pkg_name=PKG,
        path="trained_data/depth_anything_v2_metric_hypersim_vitb.onnx",
        url="https://drive.google.com/uc?id=1rM3OFvtzMrcVy56p_2GTE3ZPvLSVQjN4",
        md5="ee086caef290828a19aad4e484c282d3",
    )
    download_data(
        pkg_name=PKG,
        path="trained_data/depth_anything_v2_metric_hypersim_vitl.onnx",
        url="https://drive.google.com/uc?id=1POnPaAWHxoERc0TjmblbZPr4B4UMLWKV",
        md5="6477966396be3983214a8e3fb792274f",
    )


if __name__ == "__main__":
    main()
