"""Video: capture, discovery, quality, deinterlacing, recording, replay and the Camera thread."""
from .frame import Frame
from .quality import frame_quality, BAD_BELOW
from .capture import V4L2Source, GstSource, ListSource, has_gstreamer
from .replay import FileSource
from .record import Recorder
from .camera import Camera
from .discover import find_cameras

__all__ = ["Frame", "frame_quality", "BAD_BELOW", "V4L2Source", "GstSource", "ListSource",
           "has_gstreamer", "FileSource", "Recorder", "Camera", "find_cameras"]
