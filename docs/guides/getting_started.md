# Getting Started

## Build the docs locally

```bash
python -m pip install -r docs/requirements.txt
python scripts/generate_case_study_docs.py
make -C docs html
```

Open `docs/_build/html/index.html` in your browser.

## Live preview in a browser

```bash
python -m http.server --directory docs/_build/html 8000
```

Then visit <http://localhost:8000>.

## C++ API extraction

The docs are configured to run Doxygen automatically at build time.

```bash
doxygen --version
```

If Doxygen is not installed, the C++ API page still renders, but without extracted symbols.

## Common build flags

- `DOCS_SKIP_DOXYGEN=1 make -C docs html`: build quickly without C++ extraction
- `sphinx-build -W -b html docs docs/_build/html`: fail on warnings (recommended for CI)
- `python scripts/generate_case_study_docs.py --check`: verify generated case-study snippets are current
- `python -m pip install jsonschema && python scripts/validate_case_data.py`: validate all case data against schema
