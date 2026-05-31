# COPP Documentation

This repository contains the static documentation site for
<https://docs.copp.pro>. The source pages live in `docs/` and are built with
MkDocs Material.

## Build

Install the documentation dependencies:

```powershell
python -m pip install -r requirements.txt
```

Build the site:

```powershell
powershell -ExecutionPolicy Bypass -File tools\build-mkdocs.ps1 -Strict
```

The script uses `site/` as a temporary staging directory, then publishes the
generated static files back to the repository root. This keeps the public
`index.html` at the repository root for GitHub Pages, while avoiding the risk
of asking MkDocs to clean and write directly into the root directory.

The Rust and C API reference folders (`rust/` and `c/`) are copied into the
staging build so that links such as `/rust/v0.2.1/copp/` and `/c/v0.2.0/`
continue to work.

## Preview

After building, serve the repository root:

```powershell
python -m http.server 8000
```

Open <http://127.0.0.1:8000/> in a browser.
