"""Create an HTML-only, disposable input tree for Pagefind.

The published site is deliberately left untouched.  Doxygen pages need an
explicit Pagefind title because their visible title is a ``div`` rather than an
``h1``.  Rustdoc redirect stubs and the site's 404 page are omitted so they do
not become search results.
"""

from __future__ import annotations

import argparse
import html
from html.parser import HTMLParser
from pathlib import Path
import re
import sys


_WHITESPACE = re.compile(r"\s+")
_HEAD_CLOSE = re.compile(r"</head\s*>", re.IGNORECASE)
_VOID_ELEMENTS = {
    "area",
    "base",
    "br",
    "col",
    "embed",
    "hr",
    "img",
    "input",
    "link",
    "meta",
    "param",
    "source",
    "track",
    "wbr",
}


def _normalized_text(parts: list[str]) -> str:
    return _WHITESPACE.sub(" ", html.unescape("".join(parts))).strip()


class _DocumentInfoParser(HTMLParser):
    """Collect only the document features needed by the staging transform."""

    def __init__(self) -> None:
        super().__init__(convert_charrefs=True)
        self._stack: list[tuple[str, set[str]]] = []
        self._document_title_depth: int | None = None
        self._doxygen_title_depth: int | None = None
        self._ignored_doxygen_depth: int | None = None
        self._document_title_parts: list[str] = []
        self._doxygen_title_parts: list[str] = []
        self.generator = ""
        self.has_refresh = False
        self.has_pagefind_title = False

    @property
    def document_title(self) -> str:
        return _normalized_text(self._document_title_parts)

    @property
    def doxygen_title(self) -> str:
        return _normalized_text(self._doxygen_title_parts)

    @property
    def is_doxygen(self) -> bool:
        return self.generator.casefold().startswith("doxygen")

    def handle_starttag(
        self, tag: str, attrs: list[tuple[str, str | None]]
    ) -> None:
        tag = tag.casefold()
        attributes = {key.casefold(): value or "" for key, value in attrs}
        classes = set(attributes.get("class", "").split())

        if tag == "meta":
            if attributes.get("name", "").casefold() == "generator":
                self.generator = attributes.get("content", "")
            if attributes.get("http-equiv", "").casefold() == "refresh":
                self.has_refresh = True
            if (
                attributes.get("data-pagefind-meta", "").casefold()
                == "title[content]"
            ):
                self.has_pagefind_title = True
        elif attributes.get("data-pagefind-meta", "").casefold() == "title":
            self.has_pagefind_title = True

        if tag in _VOID_ELEMENTS:
            return

        ancestors = self._stack.copy()
        self._stack.append((tag, classes))
        depth = len(self._stack)

        if tag == "title" and self._document_title_depth is None:
            self._document_title_depth = depth

        if (
            self._doxygen_title_depth is None
            and tag == "div"
            and "title" in classes
            and any(
                "headertitle" in ancestor_classes
                for _, ancestor_classes in ancestors
            )
        ):
            self._doxygen_title_depth = depth
        elif (
            self._doxygen_title_depth is not None
            and self._ignored_doxygen_depth is None
            and classes.intersection({"ingroups", "mlabels"})
        ):
            self._ignored_doxygen_depth = depth

    def handle_startendtag(
        self, tag: str, attrs: list[tuple[str, str | None]]
    ) -> None:
        self.handle_starttag(tag, attrs)
        if tag.casefold() not in _VOID_ELEMENTS:
            self.handle_endtag(tag)

    def handle_endtag(self, tag: str) -> None:
        if not self._stack:
            return

        depth = len(self._stack)
        if self._ignored_doxygen_depth == depth:
            self._ignored_doxygen_depth = None
        if self._doxygen_title_depth == depth:
            self._doxygen_title_depth = None
        if self._document_title_depth == depth:
            self._document_title_depth = None

        # Generated documentation is well formed.  Still, tolerate an
        # unexpected close tag by unwinding to the matching element.
        tag = tag.casefold()
        while self._stack:
            open_tag, _ = self._stack.pop()
            if open_tag == tag:
                break

    def handle_data(self, data: str) -> None:
        if self._document_title_depth is not None:
            self._document_title_parts.append(data)
        if (
            self._doxygen_title_depth is not None
            and self._ignored_doxygen_depth is None
        ):
            self._doxygen_title_parts.append(data)


def _document_info(source: str) -> _DocumentInfoParser:
    parser = _DocumentInfoParser()
    parser.feed(source)
    parser.close()
    return parser


def _inject_pagefind_title(source: str, title: str) -> str:
    match = _HEAD_CLOSE.search(source)
    if match is None:
        raise ValueError("Doxygen document has no closing </head> tag")

    newline = "\r\n" if "\r\n" in source else "\n"
    escaped_title = html.escape(title, quote=True)
    metadata = (
        f'<meta data-pagefind-meta="title[content]" '
        f'content="{escaped_title}"/>{newline}'
    )
    return source[: match.start()] + metadata + source[match.start() :]


def prepare_site(
    site: Path,
    output: Path,
    *,
    include_paths: set[str] | None = None,
    exclude_paths: set[str] | None = None,
) -> dict[str, int]:
    site = site.resolve()
    output = output.resolve()
    if site == output or site in output.parents:
        raise ValueError("output must not be the source site or one of its children")
    if not site.is_dir():
        raise ValueError(f"site directory does not exist: {site}")
    if output.exists() and any(output.iterdir()):
        raise ValueError(f"output directory is not empty: {output}")

    normalized_includes = {
        Path(path).as_posix().casefold() for path in (include_paths or set())
    }
    normalized_excludes = {
        Path(path).as_posix().casefold() for path in (exclude_paths or set())
    }

    statistics = {
        "html_seen": 0,
        "html_written": 0,
        "paths_filtered": 0,
        "doxygen_titles": 0,
        "rust_redirects_skipped": 0,
        "not_found_skipped": 0,
    }

    for source_path in sorted(site.rglob("*.html")):
        statistics["html_seen"] += 1
        relative_path = source_path.relative_to(site)
        normalized_path = relative_path.as_posix().casefold()
        if (
            (normalized_includes and normalized_path not in normalized_includes)
            or normalized_path in normalized_excludes
        ):
            statistics["paths_filtered"] += 1
            continue
        relative_parts = relative_path.parts
        source = source_path.read_text(encoding="utf-8")
        info = _document_info(source)

        if relative_path.as_posix().casefold() == "404.html":
            statistics["not_found_skipped"] += 1
            continue
        if (
            relative_parts
            and relative_parts[0].casefold() == "rust"
            and info.has_refresh
        ):
            statistics["rust_redirects_skipped"] += 1
            continue

        if info.is_doxygen and not info.has_pagefind_title:
            title = info.doxygen_title or info.document_title
            if not title:
                raise ValueError(
                    f"Doxygen document has no usable title: {relative_path}"
                )
            source = _inject_pagefind_title(source, title)
            statistics["doxygen_titles"] += 1

        output_path = output / relative_path
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(source, encoding="utf-8", newline="")
        statistics["html_written"] += 1

    return statistics


def _parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--site", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--include-path",
        action="append",
        default=[],
        help="Stage only this exact site-relative HTML path (repeatable)",
    )
    parser.add_argument(
        "--exclude-path",
        action="append",
        default=[],
        help="Do not stage this exact site-relative HTML path (repeatable)",
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = _parse_args(sys.argv[1:] if argv is None else argv)
    try:
        statistics = prepare_site(
            args.site,
            args.output,
            include_paths=set(args.include_path),
            exclude_paths=set(args.exclude_path),
        )
    except (OSError, UnicodeError, ValueError) as error:
        print(f"Pagefind staging failed: {error}", file=sys.stderr)
        return 1

    print(
        "Prepared Pagefind staging: "
        f"{statistics['html_written']}/{statistics['html_seen']} HTML files; "
        f"filtered {statistics['paths_filtered']} paths; "
        f"added {statistics['doxygen_titles']} Doxygen titles; "
        f"skipped {statistics['rust_redirects_skipped']} Rust redirects and "
        f"{statistics['not_found_skipped']} not-found page."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
