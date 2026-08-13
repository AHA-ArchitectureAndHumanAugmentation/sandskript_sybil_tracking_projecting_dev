"""
Entry point for the saved-toolpath replay tool.

Run with run_replay.bat (or the sandskript conda env's python replay_main.py) →
http://localhost:5007. CONTAINED from the main app: do not run both while
connected to the robot — one RTDE controller per robot. No camera involved.

The robot brand comes from config.REPLAY_BACKEND via replay_robot.make_backend
(currently "ur"); the rest of this tool is brand-neutral.

Never `import main` here (it starts the main app's camera thread).
"""
from __future__ import annotations

import asyncio
import threading
import webbrowser

from config import HTTP_HOST, REPLAY_BACKEND, REPLAY_HTTP_PORT
from replay_robot import make_backend
from replay_server import ReplayServer

shared_state: dict = {
    "executing":  False,
    "phase":      "idle",
    "progress":   0.0,
    "exec_error": None,
}
state_lock = threading.Lock()

backend = make_backend(REPLAY_BACKEND, shared_state, state_lock)
server = ReplayServer(backend, shared_state, state_lock)


async def _main() -> None:
    asyncio.get_running_loop().call_later(
        1.0, webbrowser.open, f"http://{HTTP_HOST}:{REPLAY_HTTP_PORT}")
    await server.start()


if __name__ == "__main__":
    try:
        asyncio.run(_main())
    except KeyboardInterrupt:
        pass
    finally:
        backend.disconnect()
