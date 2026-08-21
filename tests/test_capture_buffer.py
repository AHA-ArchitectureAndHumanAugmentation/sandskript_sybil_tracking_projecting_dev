"""
The rolling buffers Capture averages, and the wait that keeps a hand out of them.

Capture does not use one frame — it averages the last DEPTH_AVERAGE_FRAMES from
each camera. So "wait for the buffer to refill" is what guarantees the picture
the robot draws from does not still contain the participant's hand, or the
projector's light on the sand.

That wait used to be `DEPTH_AVERAGE_FRAMES / DEPTH_FPS`, which is only right
while every camera frame is being caught. Profiling a two-camera rig showed the
buffers filling at 9 fps rather than 30 — the reader was sharing the canvas
loop, and that loop had no time left to spare — so 30 frames spanned 3.3 s while
the code waited 1.3. Two things fix it, and both are tested here: the reader has
its own thread, and the wait is measured from the buffers themselves.

No hardware: the camera thread is driven by writing its buffers directly, which
is exactly what the reader thread does.
"""
import threading
import time
from collections import deque

import numpy as np
import pytest

import config
from camera_thread import DepthCameraThread


NOMINAL = config.DEPTH_AVERAGE_FRAMES / config.DEPTH_FPS


def _cam(n_cameras=1):
    cam = DepthCameraThread({}, threading.Lock())
    cam._buffers = [deque(maxlen=config.DEPTH_AVERAGE_FRAMES) for _ in range(n_cameras)]
    cam._stamps = [deque(maxlen=config.DEPTH_AVERAGE_FRAMES) for _ in range(n_cameras)]
    cam._last_rgb = [None] * n_cameras
    return cam


def _fill(cam, index=0, fps=30.0, frames=None, end=None):
    """Push `frames` frames arriving at `fps`, the last one landing at `end`."""
    frames = config.DEPTH_AVERAGE_FRAMES if frames is None else frames
    end = time.monotonic() if end is None else end
    step = 1.0 / fps
    z = np.zeros((4, 4), np.float32)
    ok = np.ones((4, 4), bool)
    for k in range(frames):
        cam._buffers[index].append((z, ok))
        cam._stamps[index].append(end - (frames - 1 - k) * step)


class TestTheWaitIsMeasuredNotAssumed:

    def test_a_full_rate_buffer_reports_the_nominal_wait(self):
        """30 frames at 30 fps is one second — the value the code always used."""
        cam = _cam()
        _fill(cam, fps=30.0)
        assert cam.refill_seconds() == pytest.approx(NOMINAL, abs=0.05)

    def test_a_starved_buffer_reports_a_longer_wait(self):
        """
        The bug in one assertion. At 9 fps the same 30 frames reach 3.3 s back,
        and a wait sized for 1 s leaves two seconds of hand in the average.
        """
        cam = _cam()
        _fill(cam, fps=9.1)
        assert cam.refill_seconds() == pytest.approx(30 / 9.1, abs=0.1)
        assert cam.refill_seconds() > 3.0

    def test_the_slowest_camera_decides(self):
        """The average is only clean once EVERY buffer is."""
        cam = _cam(3)
        _fill(cam, 0, fps=30.0)
        _fill(cam, 1, fps=30.0)
        _fill(cam, 2, fps=10.0)
        assert cam.refill_seconds() == pytest.approx(3.0, abs=0.1)

    def test_it_never_returns_less_than_the_nominal_wait(self):
        """
        Measuring may only ever LENGTHEN the wait. A camera running fast, or a
        burst of frames, must not shorten it below the value the surrounding
        code was written against.
        """
        cam = _cam()
        _fill(cam, fps=120.0)
        assert cam.refill_seconds() == pytest.approx(NOMINAL)

    def test_it_is_bounded_above(self):
        """A camera that has stopped must not stall Participant Mode forever."""
        cam = _cam()
        _fill(cam, fps=0.2)
        assert cam.refill_seconds() == pytest.approx(config.CAPTURE_REFILL_MAX_S)

    def test_a_camera_that_went_quiet_is_counted(self):
        """
        Arrival history alone describes a camera that is still delivering. One
        that stopped ten seconds ago looks healthy by that measure and is not.
        """
        cam = _cam()
        _fill(cam, fps=30.0, end=time.monotonic() - 3.0)
        assert cam.refill_seconds() >= 3.0

    def test_a_partly_filled_buffer_extrapolates(self):
        """
        Mid-fill, the span so far understates the job. Five frames at 10 fps is
        0.4 s of history but 3 s of refilling still to do.
        """
        cam = _cam()
        _fill(cam, fps=10.0, frames=5)
        assert cam.refill_seconds() == pytest.approx(3.0, abs=0.1)

    def test_no_frames_at_all_falls_back_to_nominal(self):
        cam = _cam()
        assert cam.refill_seconds() == pytest.approx(NOMINAL)
        cam._buffers = cam._stamps = []
        assert cam.refill_seconds() == pytest.approx(NOMINAL)


class TestReadingIsNotPacedByTheCanvas:
    """
    The root cause. Polling shared the canvas loop, so once that loop's work
    filled its period the cameras were asked once per canvas instead of
    continuously — and the buffers filled at the canvas rate.
    """

    def test_the_reader_has_its_own_thread(self):
        import inspect

        import camera_thread

        assert hasattr(DepthCameraThread, "_reader")
        run = inspect.getsource(DepthCameraThread._run)
        assert "depth_camera_reader" in run
        assert "reader.start()" in run
        # And the canvas loop no longer polls.
        assert "realsense_source.poll" not in run

    def test_the_reader_stops_before_the_cameras_are_closed(self):
        """Polling a closed pipeline is the one ordering that must not happen."""
        import inspect

        run = inspect.getsource(DepthCameraThread._run)
        join = run.index("reader.join")
        close = run.index("realsense_source.close")
        assert join < close

    def test_the_reader_keeps_its_own_timer(self):
        """
        `+=` on a timer's slots is not atomic, so two threads sharing one lose
        samples silently.
        """
        cam = _cam()
        assert cam._read_timer is not cam._timer
        assert cam._read_timer.target_hz == pytest.approx(float(config.DEPTH_FPS))

    def test_the_reader_records_arrival_times(self):
        """Without a timestamp per frame there is nothing to measure the wait from."""
        import inspect

        src = inspect.getsource(DepthCameraThread._reader)
        assert "_stamps" in src and "time.monotonic()" in src

    def test_buffers_and_stamps_stay_the_same_length(self):
        """
        They are read together under one lock; a stamp without its frame (or the
        reverse) would silently mis-date the average.
        """
        cam = _cam(2)
        _fill(cam, 0, frames=7)
        _fill(cam, 1, frames=30)
        for buf, stamps in zip(cam._buffers, cam._stamps):
            assert len(buf) == len(stamps)
            assert buf.maxlen == stamps.maxlen == config.DEPTH_AVERAGE_FRAMES


class TestTheReaderKeepsUpWithABusyCanvas:
    """
    The property the whole fix exists for, exercised for real: run the reader
    against fake cameras while holding another thread as busy as a
    three-camera canvas is, and check the buffers still fill at camera rate.
    Under the old design this is precisely what failed.
    """

    def _run_reader(self, monkeypatch, busy_ms, duration=1.0, n=2):
        import camera_thread as ct

        cam = _cam(n)
        z = np.zeros((8, 8), np.float32)
        ok = np.ones((8, 8), bool)
        next_at = [time.monotonic()] * n
        step = 1.0 / config.DEPTH_FPS

        def fake_poll(camera):
            i = camera            # the "camera" is just its index here
            now = time.monotonic()
            if now < next_at[i]:
                return None       # nothing new yet, exactly like the SDK
            next_at[i] = max(next_at[i] + step, now - step)
            return z, ok, None

        monkeypatch.setattr(ct.realsense_source, "poll", fake_poll)

        stop_at = time.monotonic() + duration
        reader = threading.Thread(target=cam._reader, args=(list(range(n)),),
                                  daemon=True)
        reader.start()
        # Keep this thread as busy as a saturated canvas loop: long blocking
        # chunks with no yielding of its own.
        while time.monotonic() < stop_at:
            end = time.monotonic() + busy_ms / 1000.0
            while time.monotonic() < end:
                pass
        cam._stop_event.set()
        reader.join(timeout=2.0)
        return cam

    def test_buffers_fill_at_camera_rate_not_canvas_rate(self, monkeypatch):
        cam = self._run_reader(monkeypatch, busy_ms=105.0, duration=1.2)
        for stamps in cam._stamps:
            assert len(stamps) >= 2
            fps = (len(stamps) - 1) / (stamps[-1] - stamps[0])
            # Under the old design this would sit near the canvas rate (~9).
            assert fps > 20.0, f"buffer filled at only {fps:.1f} fps"

    def test_and_the_measured_wait_then_reads_nominal(self, monkeypatch):
        """The two halves of the fix agree: read at rate, and the wait is 1 s."""
        cam = self._run_reader(monkeypatch, busy_ms=105.0, duration=1.2)
        assert cam.refill_seconds() == pytest.approx(NOMINAL, abs=0.25)

    def test_the_reader_stops_promptly(self, monkeypatch):
        """It waits on the stop event, so shutdown is not paced by its sleep."""
        import camera_thread as ct

        cam = _cam(1)
        monkeypatch.setattr(ct.realsense_source, "poll", lambda c: None)
        reader = threading.Thread(target=cam._reader, args=([0],), daemon=True)
        reader.start()
        time.sleep(0.05)
        t0 = time.monotonic()
        cam._stop_event.set()
        reader.join(timeout=1.0)
        assert not reader.is_alive()
        assert time.monotonic() - t0 < 0.25


class TestThePipelineUsesTheMeasuredWait:

    def test_every_refill_wait_comes_from_the_camera_thread(self):
        """
        Participant Sensing, the projector blanking and the start-up reference
        capture all wait for the SAME buffer, so all of them must ask the same
        question. A hard-coded DEPTH_AVERAGE_FRAMES / DEPTH_FPS anywhere in
        main.py is the bug returning: those two agree only while every camera
        frame is being caught, and the derived figure is silently too short the
        moment one is missed.
        """
        from pathlib import Path

        src = Path(__file__).resolve().parents[1].joinpath("main.py").read_text(
            encoding="utf-8")
        assert "DEPTH_AVERAGE_FRAMES / DEPTH_FPS" not in src
        # Every `await asyncio.sleep(...)` that waits on the buffer must name
        # the measured value — counting call sites would just have to be
        # bumped whenever another wait is added.
        waits = [ln.strip() for ln in src.splitlines()
                 if "asyncio.sleep(" in ln and "refill" in ln]
        assert len(waits) >= 3
        assert all("camera_thread.refill_seconds()" in ln for ln in waits)

    def test_the_ceiling_is_never_below_the_nominal_wait(self):
        """Otherwise the bound would shorten every wait, which is the wrong way."""
        assert config.CAPTURE_REFILL_MAX_S >= NOMINAL
