(function () {
    "use strict";

    function stripEnglishParentheses(text) {
        return text.replace(/\s*\((?=[^)]*[A-Za-z])[^)]*\)/g, "");
    }

    function cleanupToc() {
        document.querySelectorAll(".md-nav--secondary [data-md-component='toc'] .md-nav__link").forEach(function (link) {
            var original = link.getAttribute("data-copp-toc-original") || link.textContent;
            link.setAttribute("data-copp-toc-original", original);
            link.textContent = stripEnglishParentheses(original);
        });
    }

    if (typeof window.document$ !== "undefined" && window.document$.subscribe) {
        window.document$.subscribe(cleanupToc);
    }

    if (document.readyState === "loading") {
        document.addEventListener("DOMContentLoaded", cleanupToc);
    } else {
        cleanupToc();
    }
})();
