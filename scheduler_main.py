"""
Entry point for the Scheduler tool.

Run with run_scheduler.bat (or the sandskript conda env's python
scheduler_main.py) → http://localhost:5108. Lists every saved toolpath bundle
under paths/ as a numbered spreadsheet: which path went out, and when.

CONTAINED from the main app, and READ-ONLY — no camera, no robot, nothing
written. Unlike the replay and Multi-Cam tools it therefore has nothing to
clash over, so it is safe to leave running while the main app works.

Never `import main` here (it starts the main app's camera thread).
"""
from __future__ import annotations

import asyncio
import webbrowser

from config import HTTP_HOST, SCHEDULER_HTTP_PORT
from scheduler_server import SchedulerServer

server = SchedulerServer()


async def _main() -> None:
    asyncio.get_running_loop().call_later(
        1.0, webbrowser.open, f"http://{HTTP_HOST}:{SCHEDULER_HTTP_PORT}")
    await server.start()


if __name__ == "__main__":
    try:
        asyncio.run(_main())
    except KeyboardInterrupt:
        pass
