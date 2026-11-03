# jetson worker report (draft, updated through the day)

Status: in progress. Final version at 18:30 Sydney.

## Done so far

- `grappstar` package with CLI (`python3 -m grappstar ...`): comms (C1 base client + mock base, C2 dart client + mock
  dart v4), video (V4L2/GStreamer capture with explicit FOURCC, discovery, quality, recording with timestamps,
  replay, deinterlace), vision (flight phases, terrain verdict, aerial mosaic, rover targeting, calibration),
  ballistics (C3, loads base/out/ballistics.json), mission state machine with operator prompts, operator web page,
  simulator (`serve --sim`), setup kit (install.sh, udev, systemd, README).
- Review round 1 (comms + mission): 11 findings, all fixed, regression tests in tests/test_review_round1.py.

## Open

See CONTRACT_REQUESTS.md.
