#!/usr/bin/env python3
"""Main launch script for SENTINEL Mission Control."""

import argparse
import sys
import uvicorn


def main():
    parser = argparse.ArgumentParser(description="SENTINEL Autonomous Hazard Detection & Response System")
    parser.add_argument("--host", type=str, default="0.0.0.0", help="Host address to bind (default: 0.0.0.0)")
    parser.add_argument("--port", type=int, default=8000, help="Port to listen on (default: 8000)")
    parser.add_argument("--camera-index", type=int, default=None, help="Camera device index (e.g. 1 for external USB webcam)")
    parser.add_argument("--reload", action="store_true", help="Enable auto-reload for development")

    args = parser.parse_args()

    import os
    if args.camera_index is not None:
        os.environ["SENTINEL_CAMERA_INDEX"] = str(args.camera_index)

    print("\n" + "=" * 65)
    print("      PROJECT SENTINEL — MISSION CONTROL SYSTEM ACTIVE")
    print("=" * 65)
    print(f" Web Dashboard : http://localhost:{args.port}")
    print(f" Live Stream   : http://localhost:{args.port}/video_feed")
    print(f" Telemetry WS  : ws://localhost:{args.port}/ws")
    print("=" * 65 + "\n")

    uvicorn.run(
        "sentinel.server.app:app",
        host=args.host,
        port=args.port,
        reload=args.reload,
        log_level="info",
    )


if __name__ == "__main__":
    main()
