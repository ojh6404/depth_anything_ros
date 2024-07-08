import argparse
import os
import sys
import torch

DEPTH_ANYTHING_V2_PATH = os.path.join(os.path.dirname(__file__), "..", "Depth-Anything-V2", "metric_depth")
sys.path.append(DEPTH_ANYTHING_V2_PATH)
from depth_anything_v2.dpt import DepthAnythingV2

MODEL_CONFIG = {
    "vits": {"encoder": "vits", "features": 64, "out_channels": [48, 96, 192, 384]},
    "vitb": {"encoder": "vitb", "features": 128, "out_channels": [96, 192, 384, 768]},
    "vitl": {"encoder": "vitl", "features": 256, "out_channels": [256, 512, 1024, 1024]},
    "vitg": {"encoder": "vitg", "features": 384, "out_channels": [1536, 1536, 1536, 1536]},
}


def main(args):
    depth_anything = DepthAnythingV2(**{**MODEL_CONFIG[args.model], "max_depth": args.max_depth})
    depth_anything.load_state_dict(torch.load(args.checkpoint, map_location="cpu"))
    depth_anything = depth_anything.to("cpu").eval()

    # Define dummy input data
    dummy_input = torch.ones((3, args.input_size, args.input_size)).unsqueeze(0)

    # Provide an example input to the model, this is necessary for exporting to ONNX
    example_output = depth_anything.forward(dummy_input)

    # replace the extension of the checkpoint file with .onnx
    onnx_path = args.checkpoint.replace(".pth", ".onnx")

    # Export the PyTorch model to ONNX format
    torch.onnx.export(
        depth_anything,
        dummy_input,
        onnx_path,
        opset_version=11,
        input_names=["input"],
        output_names=["output"],
        verbose=True,
    )

    print(f"Model exported to {onnx_path}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Depth Anything V2")
    parser.add_argument("--input-size", type=int, default=518)
    parser.add_argument("-m", "--model", type=str, default="vitl", choices=["vits", "vitb", "vitl", "vitg"])
    parser.add_argument(
        "-ckpt", "--checkpoint", type=str, default="trained_data/depth_anything_v2_metric_hypersim_vitl.pth"
    )
    parser.add_argument("--max-depth", type=float, default=20)
    args = parser.parse_args()
    main(args)
