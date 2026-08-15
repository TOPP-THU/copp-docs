"""Set the root HTML language from each Markdown page's metadata."""

from __future__ import annotations

import re


HTML_LANGUAGE_ATTRIBUTE = re.compile(
    r"(?P<prefix><html\b[^>]*\blang\s*=\s*)(?P<quote>['\"])(?P<value>.*?)(?P=quote)",
    re.IGNORECASE,
)
LANGUAGE_TAG = re.compile(r"^[A-Za-z]{2,3}(?:-[A-Za-z0-9]{1,8})*$")


def _page_language(page) -> str | None:
    language = page.meta.get("lang")
    if isinstance(language, (list, tuple)):
        if len(language) != 1:
            raise ValueError(f"Page {page.file.src_uri!r} must declare exactly one lang value")
        language = language[0]

    if language is None:
        return None

    language = str(language).strip()
    if not LANGUAGE_TAG.fullmatch(language):
        raise ValueError(
            f"Page {page.file.src_uri!r} has an invalid lang metadata value: {language!r}"
        )
    return language


def on_post_page(output: str, page, config) -> str:
    """Replace Material's global language with this page's declared language."""

    language = _page_language(page)
    if language is None:
        return output

    if not HTML_LANGUAGE_ATTRIBUTE.search(output):
        raise ValueError(f"Page {page.file.src_uri!r} has no root HTML lang attribute")

    return HTML_LANGUAGE_ATTRIBUTE.sub(
        lambda match: f"{match.group('prefix')}{match.group('quote')}"
        f"{language}{match.group('quote')}",
        output,
        count=1,
    )
