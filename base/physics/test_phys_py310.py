"""Every module must stay Python 3.10 compatible (Jetson JetPack 6.2, Harry's Mac)."""
import ast
import glob
import os

HERE = os.path.dirname(os.path.abspath(__file__))


def test_parses_as_python_310():
    bad = []
    for fn in sorted(glob.glob(os.path.join(HERE, "*.py"))):
        src = open(fn).read()
        try:
            ast.parse(src, filename=fn, feature_version=(3, 10))
        except SyntaxError as e:
            bad.append(f"{os.path.basename(fn)}:{e.lineno} {e.msg}")
    assert not bad, bad


def test_no_pep701_fstrings():
    """ast feature_version does not catch 3.12 f-strings that reuse the outer quote inside the braces."""
    import re
    pat = re.compile(r"""\bf"[^"\n]*\{[^}"\n]*"|\bf'[^'\n]*\{[^}'\n]*'""")
    bad = []
    for fn in sorted(glob.glob(os.path.join(HERE, "*.py"))):
        if fn.endswith("test_phys_py310.py"):
            continue
        for i, line in enumerate(open(fn), 1):
            if pat.search(line):
                bad.append(f"{os.path.basename(fn)}:{i}")
    assert not bad, bad
