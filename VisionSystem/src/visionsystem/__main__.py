"""Entry point for the perception process."""

from __future__ import annotations

import argparse


def main() -> int:
    parser = argparse.ArgumentParser(prog="visionsystem")
    parser.add_argument("--port", default="/dev/ttyACM0", help="serial port to the V5 brain")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--camera", type=int, default=0)
    args = parser.parse_args()

    raise SystemExit(f"not implemented: {args.camera=} {args.port=} {args.baud=}")


if __name__ == "__main__":
    main()
