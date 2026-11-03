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
