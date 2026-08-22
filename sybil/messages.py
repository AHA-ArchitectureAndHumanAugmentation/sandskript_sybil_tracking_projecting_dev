"""
Sybil — shared message contract between the tracking/projection process
and the RRC process.

This file must be IDENTICAL in both repos:
    sandskript_sybil_tracking_projecting_dev/sybil/messages.py
    sandskript_sybil_rrc_dev/sybil/messages.py

Do not edit one copy only. Bump PROTOCOL_VERSION whenever a field is added,
removed or renamed; both sides check it on connect, so drift fails loudly at
startup rather than silently mangling a job mid-window.

No third-party imports on purpose — this must load in both conda envs
(`sandskript` and `compas_rrc`) without installing anything.
"""

from __future__ import annotations

import json
import uuid
from dataclasses import dataclass, field, asdict
from datetime import datetime, timezone
from enum import Enum
from typing import Any, Dict, List, Optional


# ==========================================================================
# SETTINGS
# ==========================================================================

PROTOCOL_VERSION = 1
# Guards against the two repos drifting apart.
#
# This file must be IDENTICAL in both repos. If you change the SHAPE of any
# message — add a field, remove one, rename one — raise this number by 1 and
# copy the file across.
#
# Both sides check it on every message. If the numbers differ the message is
# refused with a clear error, instead of quietly arriving with missing or
# misnamed fields. That turns "something is behaving oddly and I have lost an
# hour" into "the file is out of sync, copy it across".
#
# Do not raise it for comment or formatting changes.

DEFAULT_INTERACTIONS_CAP = 4
# How many visitors may draw in one 30-minute interaction window.
# Raise it and more people get to draw, but each has less time and the
# maintenance window afterwards has more work to fit in. Lower it and the
# day is calmer with more slack. Four is what the exhibition schedule
# assumes. This is only the fallback — the exhibition script sets it.

DEFAULT_JOB_TTL_S = 120.0
# How many seconds a job stays valid after it was created.
# If the two programs lose contact and a job arrives very late, the visitor
# who drew it has long gone and the tile may have been used since. Rather
# than spray something stale, the job is thrown away and a new one asked
# for. Longer = more tolerant of slow links. Shorter = fresher, but more
# discarded jobs if the network hiccups.

# ==========================================================================


# --------------------------------------------------------------------------
# enums
# --------------------------------------------------------------------------

class TileClass(str, Enum):
    """Which kind of tile. Set once per tile in config, never changes."""
    A = "A"          # visitor drawings, substrate + water
    B = "B"          # pre-drawn GH paths, water only once the show opens


class Material(str, Enum):
    """What comes out of the nozzle. Decides speed, standoff, air setting
    and how long the tile needs before it can be sprayed again."""
    SUBSTRATE = "substrate"
    WATER = "water"


class AirSetting(str, Enum):
    SPRAY = "spray"
    MISTY = "misty"


class SessionType(str, Enum):
    LIVE = "live"
    MAINTENANCE = "maintenance"
    ROTATION = "rotation"


class PathSource(str, Enum):
    DRAWN = "drawn"              # visitor
    REPERTOIRE = "repertoire"    # saved folder, idle fallback
    SAVED_JOB = "saved_job"      # replayed for layers 2-4
    TILE_PATH = "tile_path"      # B tile's own substrate path, or rotation path


class State(str, Enum):
    """What the robot is doing right now.

    RRC broadcasts this so tracking knows whether to accept a drawing.
    A visitor cannot draw while the arm is moving — see capture_armed
    near the bottom of this file.
    """
    IDLE = "idle"                 # at home, doing nothing. Visitor may draw.
    AWAITING_JOB = "awaiting_job" # asked for a path, waiting for it to arrive
    EXECUTING = "executing"       # arm is moving. Drawings are NOT saved.
    HOLDING = "holding"           # no tile free yet. Visitor may still draw.
    FAULT = "fault"               # something went wrong. Needs a person.


class AbortReason(str, Enum):
    """Why a job ended early. Always sent with DONE, never left blank.

    If a failed job does not send DONE, the loop deadlocks: tracking waits
    forever for a completion that never arrives, and the robot sits at home
    while the window burns.
    """
    NO_GEOMETRY = "no_geometry"   # nothing left after clipping to the tile
    UNREACHABLE = "unreachable"   # the arm cannot reach one of the frames
    FAULT = "fault"               # E-stop, or the robot reported an error
    STALE = "stale"               # sat around too long to still be worth doing
    WINDOW_ENDED = "window_ended" # the clock ran out mid-job


# --------------------------------------------------------------------------
# helpers
# --------------------------------------------------------------------------

def now_iso() -> str:
    """UTC, second precision, always suffixed Z."""
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat().replace("+00:00", "Z")


def parse_iso(s: str) -> datetime:
    return datetime.fromisoformat(s.replace("Z", "+00:00"))


def age_seconds(iso_timestamp: str) -> float:
    return (datetime.now(timezone.utc) - parse_iso(iso_timestamp)).total_seconds()


def new_job_id(window: int) -> str:
    """e.g. 20260907-w1-3f9c — date, window, short random tail."""
    day = datetime.now(timezone.utc).strftime("%Y%m%d")
    return "{}-w{}-{}".format(day, window, uuid.uuid4().hex[:4])


class ProtocolMismatch(RuntimeError):
    pass


# --------------------------------------------------------------------------
# base
# --------------------------------------------------------------------------

@dataclass
class Message:
    """Common envelope. Subclasses add their own fields."""

    def to_json(self) -> str:
        payload = asdict(self)
        payload["_type"] = type(self).__name__
        payload["_version"] = PROTOCOL_VERSION
        return json.dumps(payload, default=_encode)

    @classmethod
    def from_json(cls, raw: str) -> "Message":
        payload: Dict[str, Any] = json.loads(raw)

        version = payload.pop("_version", None)
        if version != PROTOCOL_VERSION:
            raise ProtocolMismatch(
                "message protocol v{} but this process is v{} — "
                "messages.py is out of sync between the repos".format(version, PROTOCOL_VERSION)
            )

        type_name = payload.pop("_type", None)
        target = _REGISTRY.get(type_name)
        if target is None:
            raise ValueError("unknown message type: {!r}".format(type_name))

        return target(**payload)


def _encode(obj: Any) -> Any:
    if isinstance(obj, Enum):
        return obj.value
    raise TypeError("not JSON serialisable: {!r}".format(obj))


# --------------------------------------------------------------------------
# the five messages
# --------------------------------------------------------------------------

@dataclass
class PathReady(Message):
    """tracking -> rrc.  A path was tracked and saved. Nothing is projected yet."""
    timestamp: str = field(default_factory=now_iso)
    source: str = PathSource.DRAWN.value
    n_points: int = 0
    length_mm: float = 0.0


@dataclass
class TileAssigned(Message):
    """rrc -> tracking.  Which tile to project onto, and what will be sprayed."""
    job_id: str = ""
    tile_id: str = ""
    tile_class: str = TileClass.A.value
    material: str = Material.SUBSTRATE.value
    source: str = PathSource.DRAWN.value
    timestamp: str = field(default_factory=now_iso)


@dataclass
class Job(Message):
    """tracking -> rrc.  Projected frames, ready to execute.

    frames: list of [x, y, z, qw, qx, qy, qz] in robot coordinates, mm.
    Kept as plain lists so this module needs no compas import.
    """
    job_id: str = ""
    tile_id: str = ""
    material: str = Material.SUBSTRATE.value
    layer: int = 1
    frames: List[List[float]] = field(default_factory=list)
    timestamp: str = field(default_factory=now_iso)

    def is_stale(self, ttl_seconds: float = DEFAULT_JOB_TTL_S) -> bool:
        return age_seconds(self.timestamp) > ttl_seconds


@dataclass
class Done(Message):
    """rrc -> tracking.  Always sent, including on abort.

    If this is not sent the loop deadlocks: tracking waits forever for a
    completion that never arrives and the robot sits at HOME.
    """
    job_id: str = ""
    tile_id: str = ""
    layer: int = 0
    material: str = Material.SUBSTRATE.value
    duration_s: float = 0.0
    lanes_executed: int = 0
    aborted: bool = False
    abort_reason: Optional[str] = None
    timestamp: str = field(default_factory=now_iso)


@dataclass
class StateMsg(Message):
    """rrc -> tracking.  Broadcast on change AND on a heartbeat, so a missed
    transition self-corrects instead of leaving capture stuck."""
    state: str = State.IDLE.value
    session: Optional[str] = None
    window: Optional[int] = None
    interactions_used: int = 0
    interactions_cap: int = DEFAULT_INTERACTIONS_CAP
    tile_id: Optional[str] = None
    layer: Optional[int] = None
    timestamp: str = field(default_factory=now_iso)

    @property
    def capture_armed(self) -> bool:
        """The interlock. A visitor cannot draw while the arm is moving."""
        if self.state in (State.EXECUTING.value, State.FAULT.value):
            return False
        if self.interactions_used >= self.interactions_cap:
            return False
        return True

    @property
    def cap_reached(self) -> bool:
        return self.interactions_used >= self.interactions_cap


_REGISTRY = {
    c.__name__: c for c in (PathReady, TileAssigned, Job, Done, StateMsg)
}


# --------------------------------------------------------------------------
# smoke test:  python messages.py
# --------------------------------------------------------------------------

if __name__ == "__main__":
    originals = [
        PathReady(n_points=142, length_mm=1480.0),
        TileAssigned(job_id=new_job_id(1), tile_id="03", material=Material.SUBSTRATE.value),
        Job(job_id="20260907-w1-3f9c", tile_id="03", frames=[[0, 0, 0, 1, 0, 0, 0]]),
        Done(job_id="20260907-w1-3f9c", tile_id="03", layer=1, duration_s=312.4, lanes_executed=4),
        StateMsg(state=State.EXECUTING.value, window=1, interactions_used=2),
    ]

    for original in originals:
        restored = Message.from_json(original.to_json())
        assert restored == original, (original, restored)
        print("{:<14} ok".format(type(original).__name__))

    live = StateMsg(state=State.IDLE.value, interactions_used=2, interactions_cap=4)
    full = StateMsg(state=State.IDLE.value, interactions_used=4, interactions_cap=4)
    busy = StateMsg(state=State.EXECUTING.value, interactions_used=0)
    assert live.capture_armed and not full.capture_armed and not busy.capture_armed
    print("interlock      ok")

    assert Job(timestamp="2020-01-01T00:00:00Z").is_stale(60)
    print("staleness      ok")

    try:
        bad = json.loads(originals[0].to_json())
        bad["_version"] = 999
        Message.from_json(json.dumps(bad))
    except ProtocolMismatch:
        print("version guard  ok")
    else:
        raise AssertionError("version mismatch was not caught")
