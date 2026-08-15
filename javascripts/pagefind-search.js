(function () {
    "use strict";

    if (window.location.protocol === "file:") {
        return;
    }

    var state = window.__coppPagefindSearch || {
        loading: null,
        mounted: false,
        subscribed: false,
    };
    state.ui = state.ui || null;
    state.container = state.container || null;
    state.loadingContainer = state.loadingContainer || null;
    state.remountRequested = state.remountRequested || false;
    state.queryContainer = state.queryContainer || null;
    state.queryLocation = state.queryLocation || null;
    window.__coppPagefindSearch = state;

    function pagefindBaseUrl() {
        var scope = window.__md_scope || new URL(".", window.location.href);
        return new URL("pagefind/", scope).toString();
    }

    function pagefindChineseBaseUrl() {
        var scope = window.__md_scope || new URL(".", window.location.href);
        return new URL("pagefind-zh/", scope).toString();
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
                if (!document.body.contains(container)) {
                    reject(new Error("Pagefind container was replaced during navigation"));
                    return;
                }

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

    function openSearch() {
        var searchToggle = document.getElementById("__search");
        if (!searchToggle || searchToggle.checked) {
            return;
        }

        searchToggle.checked = true;
        searchToggle.dispatchEvent(new Event("change", { bubbles: true }));
    }

    function bindPagefindInput(input) {
        if (!input || input.getAttribute("data-copp-pagefind-input") === "1") {
            return;
        }

        input.setAttribute("data-copp-pagefind-input", "1");
        input.addEventListener("focus", openSearch);
        input.addEventListener("click", openSearch);
    }

    function applyUrlQuery(container, input) {
        var query;
        var locationKey = window.location.pathname + window.location.search;

        if (state.queryContainer === container && state.queryLocation === locationKey) {
            return false;
        }

        state.queryContainer = container;
        state.queryLocation = locationKey;

        try {
            query = new URL(window.location.href).searchParams.get("q");
        } catch (error) {
            return false;
        }

        if (!query || !query.trim()) {
            return false;
        }

        openSearch();

        if (state.ui && typeof state.ui.triggerSearch === "function") {
            state.ui.triggerSearch(query);
        } else {
            input.value = query;
            input.dispatchEvent(new Event("input", { bubbles: true }));
        }

        return true;
    }

    function preparePagefind(search, container, input) {
        container.hidden = false;
        search.classList.add("md-search--pagefind-ready");
        state.container = container;
        state.mounted = true;
        bindPagefindInput(input);
        applyUrlQuery(container, input);
        focusPagefindInput();
    }

    function disposePagefindUi(pagefindUi) {
        if (pagefindUi && typeof pagefindUi.destroy === "function") {
            try {
                pagefindUi.destroy();
            } catch (error) {
                console.warn("Could not dispose the previous Pagefind UI instance.", error);
            }
        }
    }

    function destroyPagefind() {
        disposePagefindUi(state.ui);

        state.ui = null;
        state.container = null;
        state.mounted = false;
    }

    function mountPagefind() {
        var search = document.querySelector(".md-search");
        var inner = document.querySelector(".md-search__inner");
        var existingContainer = document.getElementById("copp-pagefind-search");
        if (!search || !inner) {
            return;
        }
        if (existingContainer && document.body.contains(existingContainer)) {
            var existingInput = existingContainer.querySelector(".pagefind-ui__search-input");
            if (existingInput) {
                preparePagefind(search, existingContainer, existingInput);
            }
            return;
        }
        if (state.loading) {
            if (state.loadingContainer && !document.body.contains(state.loadingContainer)) {
                state.remountRequested = true;
            }
            return;
        }

        destroyPagefind();

        var container = document.createElement("div");
        container.className = "copp-pagefind";
        container.id = "copp-pagefind-search";
        container.hidden = true;
        inner.appendChild(container);
        state.loadingContainer = container;

        var base = pagefindBaseUrl();
        var pagefindUi = null;
        var abandoned = false;
        loadStylesheet(new URL("pagefind-ui.css", base).toString());
        state.loading = loadScript(new URL("pagefind-ui.js", base).toString())
            .then(function () {
                if (!window.PagefindUI) {
                    throw new Error("PagefindUI was not loaded");
                }
                if (!document.body.contains(container)) {
                    abandoned = true;
                    throw new Error("Pagefind container was replaced during navigation");
                }

                pagefindUi = new window.PagefindUI({
                    element: container,
                    showImages: false,
                    showSubResults: true,
                    resetStyles: false,
                    bundlePath: base,
                    mergeIndex: [
                        {
                            bundlePath: pagefindChineseBaseUrl(),
                            language: "zh",
                        },
                    ],
                });

                return waitForPagefindInput(container);
            })
            .then(function (input) {
                if (!document.body.contains(container)) {
                    abandoned = true;
                    throw new Error("Pagefind container was replaced during navigation");
                }

                state.ui = pagefindUi;
                preparePagefind(search, container, input);
            })
            .catch(function (error) {
                if (!document.body.contains(container)) {
                    abandoned = true;
                }
                disposePagefindUi(pagefindUi);
                if (state.ui === pagefindUi) {
                    state.ui = null;
                    state.container = null;
                }
                state.mounted = false;
                search.classList.remove("md-search--pagefind-ready");
                container.remove();
                if (!abandoned) {
                    console.warn("Pagefind search could not be initialized.", error);
                }
            })
            .then(function () {
                var shouldRemount = state.remountRequested;
                state.loading = null;
                state.loadingContainer = null;
                state.remountRequested = false;

                if (shouldRemount) {
                    mountPagefind();
                }
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
                if (document.activeElement !== input) {
                    input.focus();
                    input.select();
                }
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

    if (!state.subscribed && typeof window.document$ !== "undefined" && window.document$.subscribe) {
        state.subscribed = true;
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
