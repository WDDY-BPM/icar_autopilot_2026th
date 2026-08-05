# Camera shutdown risk

`LatestFrameCapture` gives the capture thread sole ownership of camera reads. The main
thread never calls `VideoCapture::release()` while capture is active, and shutdown
does not detach or forcibly cancel the worker.

OpenCV's V4L2 `VideoCapture::read()` is not guaranteed to return when a driver or USB
device stalls permanently. The safe fallback therefore still joins the worker and may
wait indefinitely in that hardware failure mode. A complete fix requires a Linux V4L2
capture implementation that owns the device descriptor, waits with `poll()` on both the
camera descriptor and a shutdown event descriptor, and has a real-device test that
measures bounded shutdown after a simulated stalled camera.
