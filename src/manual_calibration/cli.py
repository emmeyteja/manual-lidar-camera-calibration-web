"""Command-line entry points for datasets, previews and the local web server."""

import argparse
import logging
from pathlib import Path

import cv2

from .dataset import Project
from .demo import create_demo
from .render import render
from .server import create_server


def main():
    parser = argparse.ArgumentParser(description="Manual camera–LiDAR calibration viewer")
    commands = parser.add_subparsers(dest="command", required=True)
    demo = commands.add_parser("demo", help="Generate a synthetic example dataset")
    demo.add_argument("--output", type=Path, required=True)
    serve = commands.add_parser("serve", help="Open an existing dataset in the local viewer")
    serve.add_argument("--project", type=Path, required=True)
    serve.add_argument("--port", type=int, default=8765)
    serve.add_argument("--exports", type=Path)
    snapshot = commands.add_parser("snapshot", help="Render a baseline overlay without a browser")
    snapshot.add_argument("--project", type=Path, required=True)
    snapshot.add_argument("--output", type=Path, required=True)
    snapshot.add_argument("--frame", type=int, default=0)
    snapshot.add_argument("--camera")
    snapshot.add_argument("--recording")
    bag = commands.add_parser(
        "import-bag", help="Import sampled ROS bag images/PointCloud2; requires [bags]"
    )
    bag.add_argument("--bag", type=Path, required=True)
    bag.add_argument("--config", type=Path, required=True)
    bag.add_argument("--output", type=Path, required=True)
    bag.add_argument("--max-frames", type=int, default=120)
    bag.add_argument("--tolerance-ms", type=float, default=50)
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")
    try:
        if args.command == "demo":
            print(create_demo(args.output))
        elif args.command == "import-bag":
            from .import_bag import import_bag

            print(
                import_bag(args.bag, args.config, args.output, args.max_frames, args.tolerance_ms)
            )
        else:
            project = Project(args.project)
            if args.command == "snapshot":
                if args.output.exists():
                    raise FileExistsError("Snapshot output already exists.")
                settings = project.settings(
                    args.recording or next(iter(project.recordings)),
                    args.camera or next(iter(project.cameras)),
                )
                image, _ = render(project, settings, args.frame)
                if not cv2.imwrite(str(args.output), image):
                    raise ValueError("Could not write snapshot.")
                print(args.output)
            else:
                server = create_server(project, args.port, args.exports)
                print(f"Open http://127.0.0.1:{server.server_address[1]}", flush=True)
                try:
                    server.serve_forever()
                except KeyboardInterrupt:
                    pass
                finally:
                    server.server_close()
    except (ValueError, KeyError, OSError, ImportError) as error:
        parser.exit(1, f"Error: {error}\n")
