"""
Entry point for the Multi-Cam Vision prototype (multi-camera stitching).

Run with run_stitch.bat (or the sandskript conda env's python stitch_main.py) →
http://localhost:5006. CONTAINED from the main app: do not run both at once —
each RealSense can only be owned by one process. It uses however many cameras
are plugged in (1 … STITCH_MAX_CAMERAS); with none attached it runs on a
synthetic scene so the placement workflow can still be exercised.

Never `import main` here (it starts the main app's camera thread).
"""

from __future__ import annotations

import asyncio
import threading
import webbrowser

from config import HTTP_HOST, STITCH_HTTP_PORT, STITCH_MAX_CAMERAS
from multi_camera import MultiCameraThread, cam_key
from stitch_server import StitchServer, load_saved_calib

shared_state: dict = {
    "stitch_canvas_jpg": None,
    "stitch_info": None,
    "stitch_note": None,
    "stitch_calib": None,
}
# Per-camera thumbnails, one slot per possible camera (filled as they appear).
for _i in range(STITCH_MAX_CAMERAS):
    shared_state[cam_key(_i)] = None

state_lock = threading.Lock()

camera = MultiCameraThread(shared_state, state_lock)
server = StitchServer(camera, shared_state, state_lock)


async def _main() -> None:
    camera.set_calib(load_saved_calib())
    camera.start()
    asyncio.get_running_loop().call_later(
        1.0, webbrowser.open, f"http://{HTTP_HOST}:{STITCH_HTTP_PORT}")
    await server.start()


if __name__ == "__main__":
    try:
        asyncio.run(_main())
    except KeyboardInterrupt:
        pass
    finally:
        camera.stop()
