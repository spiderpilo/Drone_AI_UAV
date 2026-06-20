#!/usr/bin/env python3
"""
Fine-tune YOLOv8n for person detection and export to ONNX.

Usage:
  pip install ultralytics
  python3 training/train.py
  python3 training/train.py --epochs 50 --dataset coco128.yaml
"""

import argparse
from pathlib import Path
from ultralytics import YOLO


def main():
    parser = argparse.ArgumentParser(description="Fine-tune YOLOv8n for person detection")
    parser.add_argument("--epochs", type=int, default=30, help="Training epochs")
    parser.add_argument("--batch", type=int, default=16, help="Batch size")
    parser.add_argument("--imgsz", type=int, default=640, help="Image size")
    parser.add_argument("--dataset", default="coco128.yaml",
                        help="Dataset config (default: coco128 for quick test)")
    parser.add_argument("--output", default="training/output",
                        help="Directory for training results")
    args = parser.parse_args()

    output_dir = Path(args.output)
    output_dir.mkdir(parents=True, exist_ok=True)

    print("[train] Loading YOLOv8n base model...")
    model = YOLO("yolov8n.pt")

    print(f"[train] Fine-tuning on {args.dataset} for {args.epochs} epochs...")
    model.train(
        data=args.dataset,
        epochs=args.epochs,
        batch=args.batch,
        imgsz=args.imgsz,
        project=str(output_dir),
        name="person_detector",
        classes=[0],  # COCO class 0 = person only
        exist_ok=True,
    )

    print("[train] Exporting to ONNX...")
    best_pt = output_dir / "person_detector" / "weights" / "best.pt"
    tuned = YOLO(str(best_pt))
    onnx_path = tuned.export(format="onnx", imgsz=args.imgsz, simplify=True)
    print(f"[train] ONNX model saved: {onnx_path}")

    final_path = output_dir / "person_detector.onnx"
    Path(onnx_path).rename(final_path)
    print(f"[train] Final model: {final_path}")
    print("[train] Done! Copy this model into the Docker image or to the Pi.")


if __name__ == "__main__":
    main()
