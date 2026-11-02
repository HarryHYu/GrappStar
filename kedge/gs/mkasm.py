"""Assemble the self contained assembly viewer. python mkasm.py out dst.html"""
import sys, os
OUT, DST = sys.argv[1], sys.argv[2]
t = open("assembly_template.html").read()
parts = [open("../vendor/package/build/three.min.js").read(), open("../vendor/package/examples/js/controls/OrbitControls.js").read(),
         open(f"{OUT}/gs_data.js").read(), open("viewer_core.js").read(), open("assembly_app.js").read()]
libs = "".join("<script>%s</script>\n" % p.replace("</script", "<\\/script") for p in parts)
open(DST, "w").write(t.replace("<!--LIBS-->", libs))
print(DST, round(os.path.getsize(DST) / 1e6, 2), "MB")
