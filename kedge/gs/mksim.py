"""Assemble the self contained HD mission sim. python mksim.py out dst.html [dev]"""
import sys, json
OUT, DST = sys.argv[1], sys.argv[2]
dev = len(sys.argv) > 3
t = open("sim_template.html").read()
if dev:
    libs = ('<script src="../vendor/package/build/three.min.js"></script><script src="../vendor/package/examples/js/controls/OrbitControls.js"></script>'
            f'<script src="{OUT}/gs_data.js"></script><script src="{OUT}/timeline.js"></script><script src="viewer_core.js"></script><script src="sim_app.js"></script>')
    open(f"{OUT}/timeline.js", "w").write("window.TL=" + open(f"{OUT}/timeline.json").read() + ";")
else:
    parts = [open("../vendor/package/build/three.min.js").read(), open("../vendor/package/examples/js/controls/OrbitControls.js").read(),
             open(f"{OUT}/gs_data.js").read(), "window.TL=" + open(f"{OUT}/timeline.json").read() + ";",
             open("viewer_core.js").read(), open("sim_app.js").read()]
    libs = "".join("<script>%s</script>\n" % p.replace("</script", "<\\/script") for p in parts)
open(DST, "w").write(t.replace("<!--LIBS-->", libs))
import os; print(DST, round(os.path.getsize(DST) / 1e6, 2), "MB")
