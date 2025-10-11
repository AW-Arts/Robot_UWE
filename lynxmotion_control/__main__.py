from __future__ import annotations

import argparse

from .interactive import run_demo
from .serial_comm import AL5ASerialController, PrintController


def main() -> None:
    parser = argparse.ArgumentParser(description="Interactive Lynxmotion AL5A controller")
    parser.add_argument("--port", help="Serial port for SSC-32/SSC-32U controller")
    parser.add_argument(
        "--simulate",
        action="store_true",
        help="Use simulation mode (no hardware required)",
    )
    parser.add_argument(
        "--time",
        type=int,
        default=1000,
        help="Move duration in milliseconds for each command",
    )
    parser.add_argument(
        "--scan",
        action="store_true",
        help=(
            "Sweep all controller channels to help identify which pins individual "
            "servos are connected to."
        ),
    )
    args = parser.parse_args()

    if args.simulate or not args.port:
        controller = PrintController()
    else:
        controller = AL5ASerialController(args.port)

    if args.scan:
        controller.scan_channels()
        return

    run_demo(controller, move_time_ms=args.time)


if __name__ == "__main__":
    main()
