import tempfile
import unittest
from pathlib import Path

from tools.prepare_pagefind_site import prepare_site


DOXYGEN_PAGE = """<!doctype html>
<html><head>
<meta name="generator" content="Doxygen 1.17.0"/>
<title>COPP C ABI: Fallback title</title>
</head><body>
<div class="headertitle"><div class="title">Topp3Problem Struct Reference
<div class="ingroups"><a>Copp formulation</a></div></div></div>
<h1>Example</h1>
</body></html>
"""

RUST_REDIRECT = """<!doctype html><html><head>
<meta http-equiv="refresh" content="0;URL=target.html">
<title>Redirection</title></head><body>Redirecting</body></html>
"""


class PreparePagefindSiteTests(unittest.TestCase):
    def test_stages_titles_and_omits_search_noise(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            site = root / "site"
            output = root / "pagefind-site"
            (site / "c").mkdir(parents=True)
            (site / "rust" / "v1").mkdir(parents=True)
            (site / "guide").mkdir(parents=True)
            (site / "c" / "problem.html").write_text(
                DOXYGEN_PAGE, encoding="utf-8"
            )
            (site / "rust" / "v1" / "redirect.html").write_text(
                RUST_REDIRECT, encoding="utf-8"
            )
            (site / "404.html").write_text("not found", encoding="utf-8")
            normal_page = "<html><head><title>Guide</title></head><body>Text</body></html>"
            (site / "guide" / "index.html").write_text(
                normal_page, encoding="utf-8"
            )

            statistics = prepare_site(site, output)

            staged_doxygen = (output / "c" / "problem.html").read_text(
                encoding="utf-8"
            )
            self.assertIn(
                'data-pagefind-meta="title[content]" '
                'content="Topp3Problem Struct Reference"',
                staged_doxygen,
            )
            self.assertNotIn("Copp formulation\"/>", staged_doxygen)
            self.assertEqual(
                DOXYGEN_PAGE,
                (site / "c" / "problem.html").read_text(encoding="utf-8"),
            )
            self.assertFalse((output / "rust" / "v1" / "redirect.html").exists())
            self.assertFalse((output / "404.html").exists())
            self.assertEqual(
                normal_page,
                (output / "guide" / "index.html").read_text(encoding="utf-8"),
            )
            self.assertEqual(
                {
                    "html_seen": 4,
                    "html_written": 2,
                    "paths_filtered": 0,
                    "doxygen_titles": 1,
                    "rust_redirects_skipped": 1,
                    "not_found_skipped": 1,
                },
                statistics,
            )

    def test_exact_path_filters_support_separate_language_indexes(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            site = root / "site"
            site.mkdir()
            for name in ("index.html", "index-zh.html", "other.html"):
                (site / name).write_text(
                    f"<html><body>{name}</body></html>", encoding="utf-8"
                )

            english_output = root / "english"
            english_statistics = prepare_site(
                site, english_output, exclude_paths={"index-zh.html"}
            )
            self.assertFalse((english_output / "index-zh.html").exists())
            self.assertEqual(1, english_statistics["paths_filtered"])

            chinese_output = root / "chinese"
            chinese_statistics = prepare_site(
                site, chinese_output, include_paths={"index-zh.html"}
            )
            self.assertTrue((chinese_output / "index-zh.html").exists())
            self.assertFalse((chinese_output / "index.html").exists())
            self.assertEqual(2, chinese_statistics["paths_filtered"])

    def test_rejects_output_inside_source_site(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            site = Path(temporary_directory) / "site"
            site.mkdir()
            with self.assertRaisesRegex(ValueError, "must not be"):
                prepare_site(site, site / "staging")


if __name__ == "__main__":
    unittest.main()
