# COPP Documentation

This repository contains the static documentation site for <https://docs.copp.pro>. The source pages live in `docs/` and are built with MkDocs Material.

## Build

Install the documentation dependencies:

```powershell
python -m pip install -r requirements.txt
```

This installs MkDocs Material and the Pagefind extended binary used to build the full-site search index, including Chinese text and the copied API reference pages.

Build the site:

```powershell
powershell -ExecutionPolicy Bypass -File tools\build-mkdocs.ps1 -Strict
```

The script uses `site/` as a temporary staging directory, then publishes the generated static files back to the repository root. This keeps the public `index.html` at the repository root for GitHub Pages, while avoiding the risk of asking MkDocs to clean and write directly into the root directory.

The Rust, C, C++, Python, and MATLAB API reference folders are copied into the staging build so that versioned reference links continue to work. Pagefind then builds an English full-site index under `pagefind/` and a Chinese index under `pagefind-zh/`; the browser search merges both indexes. Redirect stubs and the 404 page are omitted from search, and Doxygen titles are normalized in a disposable indexing tree without modifying the published API pages.

## Preview

After building, serve the repository root:

```powershell
python -m http.server 8000
```

Open <http://127.0.0.1:8000/> in a browser.
