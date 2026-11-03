"""SerialTransport open order (no ESP32 reset when the port opens)."""
def test_serial_open_sets_lines_low_before_open(monkeypatch):
    """r2_J #11: the port must be configured with RTS and DTR low before it is opened (no ESP32 reset on open)."""
    import serial
    from grappstar.comms.base import SerialTransport
    log = []

    class FakeSerial:
        def __init__(self, *a, **k):
            assert not a and not k, "Serial must be created unopened"
            self.is_open = False
        def __setattr__(self, k, v):
            if k in ("rts", "dtr", "port"):
                log.append((k, v, self.__dict__.get("is_open", False)))
            object.__setattr__(self, k, v)
        def open(self):
            log.append(("open",))
            object.__setattr__(self, "is_open", True)
        def close(self):
            pass
    monkeypatch.setattr(serial, "Serial", FakeSerial)
    t = SerialTransport("/dev/fake")
    t.open()
    names = [e[0] for e in log]
    assert names.index("rts") < names.index("open") and names.index("dtr") < names.index("open")
    vals = {e[0]: e[1] for e in log if e[0] in ("rts", "dtr")}
    assert vals == {"dtr": True, "rts": False}       # never DTR low with RTS high (the reset state)


def test_link_up_only_after_a_c1_line():
    """r3_N #7: a board that never speaks C1 (the dart's XIAO has the same USB id) never counts as the base, so no
    command is ever written to it."""
    from grappstar.comms.base import BaseClient, BaseDisconnected, pipe_pair
    import pytest
    a, b = pipe_pair()
    c = BaseClient(b, reconnect_s=0.05).start()
    try:
        a.write(b"ESP-ROM:esp32s3-20210327\nkedge dart v4 ready\n{\"hello\": 1}\n")   # not C1
        assert not c.wait_connected(0.5)
        with pytest.raises(BaseDisconnected):
            c.ping(timeout=0.2)
        assert not a.rx                                     # nothing was sent to the wrong board
        a.write(b'{"tel":{"state":"IDLE"}}\n')
        assert c.wait_connected(1.0)
    finally:
        c.close()


def test_second_open_of_the_same_port_is_refused_clearly():
    """docs learner round: serve, base monitor and base cmd on one tty split the lines. The port opens exclusively
    and a second open says the port is busy."""
    import os
    import pty
    import pytest
    from grappstar.comms.base import PortBusy, SerialTransport
    master, slave = pty.openpty()
    path = os.ttyname(slave)
    a, b = SerialTransport(path), SerialTransport(path)
    try:
        a.open()
        with pytest.raises(PortBusy) as ei:
            b.open()
        assert "busy" in str(ei.value) and path in str(ei.value)
        a.close()
        b.open()                     # free again once the first one closed
        b.close()
    finally:
        os.close(master)
        os.close(slave)
