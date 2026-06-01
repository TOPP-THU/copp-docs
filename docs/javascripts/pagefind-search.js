(function () {
    "use strict";

    var mounted = false;

    if (window.location.protocol === "file:") {
        return;
    }

    function pagefindBaseUrl() {
        var scope = window.__md_scope || new URL(".", window.location.href);
        return new URL("pagefind/", scope).toString();
    }

    function loadStylesheet(href) {
        if (document.querySelector('link[data-copp-pagefind="style"]')) {
            return;
        }

        var link = document.createElement("link");
        link.rel = "stylesheet";
        link.href = href;
        link.setAttribute("data-copp-pagefind", "style");
        document.head.appendChild(link);
    }

    function loadScript(src) {
        return new Promise(function (resolve, reject) {
            var existing = document.querySelector('script[data-copp-pagefind="script"]');
            if (existing) {
                if (window.PagefindUI) {
                    resolve();
                } else {
                    existing.addEventListener("load", resolve, { once: true });
                    existing.addEventListener("error", reject, { once: true });
                }
                return;
            }

            var script = document.createElement("script");
            script.src = src;
            script.defer = true;
            script.setAttribute("data-copp-pagefind", "script");
            script.addEventListener("load", resolve, { once: true });
            script.addEventListener("error", reject, { once: true });
            document.body.appendChild(script);
        });
    }

    function waitForPagefindInput(container) {
        return new Promise(function (resolve, reject) {
            var attempts = 40;

            function check() {
                var input = container.querySelector(".pagefind-ui__search-input");
                if (input) {
                    resolve(input);
                    return;
                }

                attempts -= 1;
                if (attempts <= 0) {
                    reject(new Error("Pagefind input was not rendered"));
                    return;
                }

                window.setTimeout(check, 25);
            }

            check();
        });
    }

    function mountPagefind() {
        var search = document.querySelector(".md-search");
        var inner = document.querySelector(".md-search__inner");
        var existingContainer = document.getElementById("copp-pagefind-search");
        if (mounted && existingContainer && document.body.contains(existingContainer)) {
            return;
        }
        if (!search || !inner) {
            return;
        }

        mounted = true;

        var container = document.createElement("div");
        container.className = "copp-pagefind";
        container.id = "copp-pagefind-search";
        container.hidden = true;
        inner.appendChild(container);

        var base = pagefindBaseUrl();
        loadStylesheet(new URL("pagefind-ui.css", base).toString());
        loadScript(new URL("pagefind-ui.js", base).toString())
            .then(function () {
                if (!window.PagefindUI) {
                    throw new Error("PagefindUI was not loaded");
                }

                new window.PagefindUI({
                    element: "#copp-pagefind-search",
                    showImages: false,
                    showSubResults: true,
                    resetStyles: false,
                    bundlePath: base,
                });

                return waitForPagefindInput(container);
            })
            .then(function () {
                container.hidden = false;
                search.classList.add("md-search--pagefind-ready");
                focusPagefindInput();
            })
            .catch(function () {
                mounted = false;
                search.classList.remove("md-search--pagefind-ready");
                container.remove();
            });
    }

    function focusPagefindInput() {
        var searchToggle = document.getElementById("__search");
        if (!searchToggle || !searchToggle.checked) {
            return;
        }

        window.setTimeout(function () {
            var input = document.querySelector("#copp-pagefind-search .pagefind-ui__search-input");
            if (input) {
                input.focus();
            }
        }, 50);
    }

    function bindSearchToggle() {
        var searchToggle = document.getElementById("__search");
        if (!searchToggle || searchToggle.getAttribute("data-copp-pagefind-toggle") === "1") {
            return;
        }

        searchToggle.setAttribute("data-copp-pagefind-toggle", "1");
        searchToggle.addEventListener("change", focusPagefindInput);
    }

    if (typeof window.document$ !== "undefined" && window.document$.subscribe) {
        window.document$.subscribe(function () {
            mountPagefind();
            bindSearchToggle();
        });
    }

    if (document.readyState === "loading") {
        document.addEventListener("DOMContentLoaded", function () {
            mountPagefind();
            bindSearchToggle();
        });
    } else {
        mountPagefind();
        bindSearchToggle();
    }
})();
