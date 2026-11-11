"""Loopback-only HTTP service. Browser requests cannot choose filesystem paths."""

import copy
import hmac
import json
import logging
import secrets
import threading
import uuid
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from importlib.resources import files
from pathlib import Path
from urllib.parse import parse_qs, urlparse

import cv2

from .calibration import editable_settings, export_settings
from .render import render, write_video

LOGGER = logging.getLogger(__name__)


def create_server(project, port=8765, exports=None):
    """Construct a server for CLI use or tests; port=0 allocates a free port."""
    token = secrets.token_urlsafe(32)
    export_dir = Path(exports).resolve() if exports else project.root / "exports"
    jobs = {}
    lock = threading.Lock()
    static = files("manual_calibration").joinpath("static")

    class Handler(BaseHTTPRequestHandler):
        def send(self, data, kind="application/json", status=200, headers=None):
            if isinstance(data, (dict, list)):
                data = json.dumps(data, allow_nan=False).encode()
            if isinstance(data, str):
                data = data.encode()
            self.send_response(status)
            for key, value in {
                "Content-Type": kind,
                "Content-Length": str(len(data)),
                "Cache-Control": "no-store",
                "X-Content-Type-Options": "nosniff",
                "Referrer-Policy": "no-referrer",
                **(headers or {}),
            }.items():
                self.send_header(key, value)
            self.end_headers()
            try:
                self.wfile.write(data)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def allowed_host(self):
            port = self.server.server_address[1]
            allowed = {f"127.0.0.1:{port}", f"localhost:{port}"}
            if self.headers.get("Host") not in allowed:
                self.send({"error": "Use the localhost address printed by the server."}, status=403)
                return False
            return True

        def do_GET(self):
            if not self.allowed_host():
                return
            parsed = urlparse(self.path)
            try:
                if parsed.path == "/":
                    html = (
                        static.joinpath("index.html")
                        .read_text()
                        .replace("__SESSION_TOKEN__", token)
                    )
                    self.send(
                        html,
                        "text/html; charset=utf-8",
                        headers={
                            "Content-Security-Policy": "default-src 'self'; img-src 'self' blob:; "
                            "script-src 'self'; style-src 'self'; connect-src 'self'; "
                            "object-src 'none'; base-uri 'none'; frame-ancestors 'none'"
                        },
                    )
                elif parsed.path in ("/app.js", "/style.css"):
                    name = parsed.path[1:]
                    mime = "text/javascript" if name.endswith(".js") else "text/css"
                    self.send(static.joinpath(name).read_bytes(), mime)
                elif parsed.path == "/config":
                    query = parse_qs(parsed.query)
                    rid = query.get("recording", [next(iter(project.recordings))])[0]
                    cid = query.get("camera", [next(iter(project.cameras))])[0]
                    self.send(project.configuration(rid, cid))
                elif parsed.path == "/jobs":
                    with lock:
                        self.send(
                            {
                                key: {k: v for k, v in job.items() if k != "path"}
                                for key, job in jobs.items()
                            }
                        )
                elif parsed.path.startswith("/video-file/"):
                    identifier = parsed.path.rsplit("/", 1)[-1]
                    with lock:
                        job = copy.deepcopy(jobs.get(identifier, {}))
                    if job.get("status") != "complete":
                        self.send({"error": "Video not available."}, status=404)
                        return
                    target = job["path"]
                    self.send_response(200)
                    self.send_header("Content-Type", "video/mp4")
                    self.send_header("Content-Length", str(target.stat().st_size))
                    self.send_header(
                        "Content-Disposition", 'attachment; filename="manual-overlay.mp4"'
                    )
                    self.end_headers()
                    with target.open("rb") as stream:
                        while chunk := stream.read(1024 * 1024):
                            self.wfile.write(chunk)
                else:
                    self.send({"error": "Not found."}, status=404)
            except (KeyError, ValueError, TypeError):
                self.send(
                    {"error": "Unknown recording/camera or invalid project data."}, status=400
                )
            except (BrokenPipeError, ConnectionResetError):
                pass
            except Exception:
                LOGGER.exception("GET request failed")
                self.send(
                    {"error": "Could not read project data; inspect the server log."}, status=500
                )

        def do_POST(self):
            if not self.allowed_host():
                return
            if not hmac.compare_digest(self.headers.get("X-Calibration-Token", ""), token):
                self.send({"error": "Reload the page to obtain a valid session token."}, status=403)
                return
            try:
                if self.headers.get_content_type() != "application/json":
                    raise ValueError("Send application/json requests.")
                length = int(self.headers.get("Content-Length", "0"))
                if not 0 < length <= 100000:
                    raise ValueError("Invalid request size (maximum 100 KB).")
                request = json.loads(self.rfile.read(length))
                submitted = request["settings"]
                rid, cid = submitted["recording_id"], submitted["camera_id"]
                settings = editable_settings(project.settings(rid, cid), submitted)
                path = urlparse(self.path).path
                if path == "/preview":
                    image, info = render(
                        project,
                        settings,
                        request["frame"],
                        request.get("mode", "original_overlay"),
                        request.get("width", 960),
                    )
                    ok, encoded = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 92])
                    if not ok:
                        raise ValueError("Could not encode preview.")
                    self.send(
                        encoded.tobytes(),
                        "image/jpeg",
                        headers={"X-Overlay-Info": json.dumps(info)},
                    )
                elif path == "/export":
                    self.send(
                        export_settings(settings),
                        headers={
                            "Content-Disposition": 'attachment; filename="manual-calibration.json"'
                        },
                    )
                elif path == "/video":
                    with lock:
                        if any(job["status"] == "rendering" for job in jobs.values()):
                            raise ValueError("A video is rendering; wait for it to finish.")
                        identifier = uuid.uuid4().hex
                        target = export_dir / f"overlay-{identifier}.mp4"
                        while len(jobs) >= 32:
                            jobs.pop(next(iter(jobs)))
                        jobs[identifier] = {"status": "rendering"}

                    def encode():
                        try:
                            write_video(project, settings, target)
                            result = {
                                "status": "complete",
                                "path": target,
                                "url": f"/video-file/{identifier}",
                            }
                        except Exception:
                            LOGGER.exception("Video export failed")
                            result = {
                                "status": "failed",
                                "error": "Export failed; check FFmpeg and the server log.",
                            }
                        with lock:
                            jobs[identifier] = result

                    threading.Thread(target=encode, daemon=True).start()
                    self.send({"job": identifier}, status=202)
                else:
                    self.send({"error": "Not found."}, status=404)
            except (KeyError, ValueError, TypeError, OverflowError) as error:
                self.send({"error": str(error)}, status=400)
            except Exception:
                LOGGER.exception("POST request failed")
                self.send({"error": "Request failed; inspect the server log."}, status=500)

        def log_message(self, format, *args):
            LOGGER.debug(format, *args)

    return ThreadingHTTPServer(("127.0.0.1", port), Handler)
