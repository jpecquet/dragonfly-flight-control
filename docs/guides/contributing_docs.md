# Contributing Docs

## Principles

- Keep formulas close to implementation and cite source files.
- Prefer explicit variable definitions over prose-only descriptions.
- When behavior changes, update docs in the same PR as code.

## Where to add content

- Theory and modeling: `docs/theory/`
- Data/output formats: `docs/output_format.md`
- Tool and API usage: `docs/api/`

## Style

- Use MyST Markdown.
- Use LaTeX math blocks for equations.
- Include code references with file paths, for example: `src/wing.cpp`.

## Media Regeneration

- Docs media artifacts are tracked in `docs/media_registry.json`.
- Use `python scripts/update_docs_media.py --list` to inspect registered media groups.
- Use `python scripts/update_docs_media.py` to regenerate and sync all registered media.
- Use `python scripts/update_docs_media.py --only <entry_id>` to update a subset.
