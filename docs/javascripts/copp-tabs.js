(function () {
    "use strict";

    var STORAGE_KEY = "copp.docs.language-tab";
    var LABELS = ["Rust", "C", "Python"];

    function normalizeLabel(value) {
        return String(value || "").trim();
    }

    function isLanguageLabel(value) {
        return LABELS.indexOf(normalizeLabel(value)) >= 0;
    }

    function storedLabel() {
        try {
            var own = window.localStorage.getItem(STORAGE_KEY);
            if (isLanguageLabel(own)) {
                return normalizeLabel(own);
            }
        } catch (error) {
            return "";
        }

        if (typeof window.__md_get === "function") {
            var materialTabs = window.__md_get("__tabs");
            if (Array.isArray(materialTabs)) {
                for (var i = 0; i < materialTabs.length; i += 1) {
                    if (isLanguageLabel(materialTabs[i])) {
                        return normalizeLabel(materialTabs[i]);
                    }
                }
            }
        }

        return "";
    }

    function rememberLabel(label) {
        try {
            window.localStorage.setItem(STORAGE_KEY, label);
        } catch (error) {
            // Ignore storage errors in restricted browsing contexts.
        }

        if (typeof window.__md_set === "function") {
            window.__md_set("__tabs", [label]);
        }
    }

    function activateTabs(label, remember) {
        var active = normalizeLabel(label);
        if (!isLanguageLabel(active)) {
            return;
        }

        document.querySelectorAll(".tabbed-set").forEach(function (set) {
            var labels = Array.prototype.slice.call(set.querySelectorAll(".tabbed-labels > label"));
            var target = labels.find(function (item) {
                return normalizeLabel(item.textContent) === active;
            });
            if (!target) {
                return;
            }

            var input = document.getElementById(target.htmlFor);
            if (input) {
                input.checked = true;
            }
        });

        if (remember) {
            rememberLabel(active);
        }
    }

    function bindTabs() {
        document.addEventListener("click", function (event) {
            var label = event.target.closest(".tabbed-labels > label");
            if (!label || !isLanguageLabel(label.textContent)) {
                return;
            }

            window.requestAnimationFrame(function () {
                activateTabs(label.textContent, true);
            });
        });
    }

    function initTabs() {
        activateTabs(storedLabel(), false);
    }

    bindTabs();

    if (typeof window.document$ !== "undefined" && window.document$.subscribe) {
        window.document$.subscribe(initTabs);
    } else if (document.readyState === "loading") {
        document.addEventListener("DOMContentLoaded", initTabs);
    } else {
        initTabs();
    }
})();
