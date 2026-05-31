"""Hide MkDocs link warnings for static API docs copied after build."""

from __future__ import annotations

import logging
import re


STATIC_API_LINK_WARNING = re.compile(
    r"(?:link|url|target|path to) ['\"]/?(?:c|rust|python)/",
    re.IGNORECASE,
)


class StaticApiLinkWarningFilter(logging.Filter):
    def filter(self, record: logging.LogRecord) -> bool:
        message = record.getMessage()
        return not STATIC_API_LINK_WARNING.search(message)


def on_config(config):
    link_filter = StaticApiLinkWarningFilter()

    for logger_name in (
        "mkdocs",
        "mkdocs.config",
        "mkdocs.structure.nav",
        "mkdocs.structure.pages",
    ):
        logger = logging.getLogger(logger_name)
        logger.addFilter(link_filter)
        for handler in logger.handlers:
            handler.addFilter(link_filter)

    for handler in logging.getLogger().handlers:
        handler.addFilter(link_filter)

    return config
