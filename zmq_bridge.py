"""
zmq_bridge.py -- talks to Charlotte's robot pipeline over ZeroMQ.

Two one-way channels:
    send_path_to_charlotte(folder, tile_id)  -- after a save, push path.json
                                                 to Charlotte, port 5557.
    TileReceiver                             -- background thread, receives
                                                 the next tile ID from
                                                 Charlotte, port 5558.

Units: this app works in metres. Charlotte's pipeline works in
millimetres. That conversion happens here, once, on the way out.

Message payload is the full JSON content as a string, not a file path --
a path only means anything on the machine that wrote it.
"""
from __future__ import annotations

import json
import threading
from pathlib import Path
from typing import Optional

import zmq

CHARLOTTE_ADDRESS = "tcp://127.0.0.1:5557"   # TODO: Charlotte's real IP, once off localhost
RECEIVE_ADDRESS = "tcp://*:5558"

LINGER_MS = 1000
METRES_TO_MM = 1000.0


def send_path_to_charlotte(folder: Path, tile_id: int, address: str = CHARLOTTE_ADDRESS) -> None:
    """Reads folder/path.json, converts metres to millimetres, adds
    tile_id at the top level, and pushes the resulting JSON as a string."""
    data = json.loads((folder / "path.json").read_text(encoding="utf-8-sig"))

    for stroke in data.get("strokes", []):
        for point in stroke:
            plane = point.get("plane")
            if plane and "origin" in plane:
                plane["origin"] = [v * METRES_TO_MM for v in plane["origin"]]

    data["tile_id"] = tile_id

    context = zmq.Context()
    sender = context.socket(zmq.PUSH)
    sender.setsockopt(zmq.LINGER, LINGER_MS)
    sender.connect(address)
    sender.send_string(json.dumps(data))
    sender.close()
    context.term()


class TileReceiver:
    """Background thread. Writes the received tile ID into shared_state
    under state_lock; nothing polls this thread directly."""

    def __init__(self, shared_state: dict, state_lock: threading.Lock,
                 address: str = RECEIVE_ADDRESS) -> None:
        self._shared_state = shared_state
        self._state_lock = state_lock
        self._address = address
        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()

    def start(self) -> None:
        if self._thread is not None:
            return
        self._thread = threading.Thread(target=self._run, daemon=True, name="zmq_tile_receiver")
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()

    def _run(self) -> None:
        context = zmq.Context()
        receiver = context.socket(zmq.PULL)
        receiver.bind(self._address)
        receiver.RCVTIMEO = 500

        while not self._stop_event.is_set():
            try:
                tile_id = receiver.recv_string()
            except zmq.Again:
                continue
            with self._state_lock:
                self._shared_state["next_tile_id"] = int(tile_id)
                self._shared_state["next_tile_seq"] = self._shared_state.get("next_tile_seq", 0) + 1
            print(f"[zmq_bridge] Received next tile: {tile_id}")

        receiver.close()
        context.term()