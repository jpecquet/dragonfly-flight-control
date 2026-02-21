from __future__ import annotations

import os
import shutil
import subprocess
import sys
from datetime import date
from pathlib import Path

DOCS_DIR = Path(__file__).resolve().parent
REPO_ROOT = DOCS_DIR.parent
sys.path.insert(0, str(REPO_ROOT))

project = "Dragonfly Flight Control"
author = "Jean Pecquet"
copyright = f"{date.today().year}, {author}"

extensions = [
    "myst_parser",
    "sphinx.ext.autodoc",
    "sphinx.ext.napoleon",
    "sphinx.ext.mathjax",
    "sphinx.ext.ifconfig",
    "sphinx.ext.viewcode",
    "sphinx.ext.intersphinx",
    "sphinx.ext.todo",
    "breathe",
    "sphinxcontrib.bibtex",
]

source_suffix = {
    ".rst": "restructuredtext",
    ".md": "markdown",
}

master_doc = "index"
templates_path = ["_templates"]
exclude_patterns = [
    "_build",
    "Thumbs.db",
    ".DS_Store",
    "validation/generated/*.md",
]

html_theme = "furo"
html_title = "Dragonfly Flight Control Docs"
html_static_path = ["_static"]
html_css_files = ["custom.css"]
html_js_files = ["theme_media.js"]
html_show_sphinx = False

bibtex_bibfiles = ["references.bib"]

myst_enable_extensions = [
    "amsmath",
    "dollarmath",
    "colon_fence",
    "deflist",
    "substitution",
]
myst_heading_anchors = 3

autodoc_member_order = "bysource"
autodoc_typehints = "description"
autodoc_default_options = {
    "members": True,
    "undoc-members": True,
    "show-inheritance": True,
}
autodoc_mock_imports = [
    "numpy",
    "matplotlib",
    "h5py",
    "scipy",
    "mpl_toolkits",
]

intersphinx_mapping = {
    "python": ("https://docs.python.org/3", None),
    "numpy": ("https://numpy.org/doc/stable", None),
}
if os.environ.get("DOCS_ENABLE_INTERSPHINX", "0") != "1":
    intersphinx_mapping = {}

todo_include_todos = True
suppress_warnings = [
    "bibtex.duplicate_citation",
]

cpp_api_available = False

def run_doxygen() -> bool:
    doxygen = shutil.which("doxygen")
    doxyfile = DOCS_DIR / "Doxyfile"

    if doxygen is None:
        print("[docs] Doxygen not found; C++ API page will show setup instructions.")
        return False
    if not doxyfile.exists():
        print(f"[docs] Doxyfile not found at {doxyfile}; skipping C++ API extraction.")
        return False

    (DOCS_DIR / "_build" / "doxygen").mkdir(parents=True, exist_ok=True)
    subprocess.run([doxygen, str(doxyfile)], cwd=str(DOCS_DIR), check=True)
    return True


if os.environ.get("DOCS_SKIP_DOXYGEN", "0") != "1":
    try:
        cpp_api_available = run_doxygen()
    except Exception as exc:
        print(f"[docs] Doxygen failed: {exc}")
        cpp_api_available = False

breathe_projects = {
    "dragonfly": str(DOCS_DIR / "_build" / "doxygen" / "xml"),
}
breathe_default_project = "dragonfly"
breathe_domain_by_extension = {
    "h": "cpp",
    "hpp": "cpp",
    "cpp": "cpp",
}


def setup(app):
    app.add_config_value("cpp_api_available", cpp_api_available, "env")
