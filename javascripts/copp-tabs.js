(function () {
    "use strict";

    var GROUPS = [
        {
            key: "language",
            storageKey: "copp.docs.language-tab",
            labels: ["Rust", "Python", "Matlab", "C++", "C"],
        },
        {
            key: "platform",
            storageKey: "copp.docs.platform-tab",
            labels: ["Linux", "Windows", "macOS", "Linux / macOS"],
            aliases: {
                "Linux / macOS": "Linux",
            },
        },
    ];

    function normalizeLabel(value) {
        return String(value || "").trim();
    }

    function canonicalLabel(group, value) {
        var label = normalizeLabel(value);
        return (group.aliases && group.aliases[label]) || label;
    }

    function groupForLabel(value) {
        var label = normalizeLabel(value);
        for (var i = 0; i < GROUPS.length; i += 1) {
            var group = GROUPS[i];
            if (group.labels.indexOf(label) >= 0) {
                return group;
            }
        }
        return null;
    }

    function isGroupLabel(group, value) {
        return group.labels.indexOf(normalizeLabel(value)) >= 0;
    }

    function storedLabels() {
        var stored = {};

        GROUPS.forEach(function (group) {
            try {
                var own = window.localStorage.getItem(group.storageKey);
                if (isGroupLabel(group, own)) {
                    stored[group.key] = canonicalLabel(group, own);
                }
            } catch (error) {
                // Ignore storage errors in restricted browsing contexts.
            }
        });

        try {
            if (typeof window.__md_get === "function") {
                var materialTabs = window.__md_get("__tabs");
                if (Array.isArray(materialTabs)) {
                    materialTabs.forEach(function (tab) {
                        var group = groupForLabel(tab);
                        if (group && !stored[group.key]) {
                            stored[group.key] = canonicalLabel(group, tab);
                        }
                    });
                }
            }
        } catch (error) {
            // Ignore malformed Material storage.
        }

        return stored;
    }

    function rememberLabel(group, label) {
        try {
            window.localStorage.setItem(group.storageKey, label);
        } catch (error) {
            // Ignore storage errors in restricted browsing contexts.
        }

        if (typeof window.__md_set === "function") {
            var tabs = [];
            try {
                var materialTabs = window.__md_get("__tabs");
                if (Array.isArray(materialTabs)) {
                    tabs = materialTabs.filter(function (tab) {
                        var tabGroup = groupForLabel(tab);
                        return !tabGroup || tabGroup.key !== group.key;
                    });
                }
            } catch (error) {
                tabs = [];
            }
            tabs.push(label);
            window.__md_set("__tabs", tabs);
        }
    }

    function setActiveLabelStyle(label, active) {
        label.classList.toggle("is-active", active);
        label.style.background = active ? "#1f6feb" : "";
        label.style.color = active ? "#fff" : "";
        label.style.fontWeight = active ? "700" : "";
    }

    function activateTabs(group, label, remember) {
        var active = canonicalLabel(group, label);
        if (!isGroupLabel(group, active)) {
            return;
        }

        document.querySelectorAll(".tabbed-set").forEach(function (set) {
            var labels = Array.prototype.slice.call(set.querySelectorAll(".tabbed-labels > label"));
            var target = labels.find(function (item) {
                return groupForLabel(item.textContent) === group && canonicalLabel(group, item.textContent) === active;
            });
            if (!target) {
                return;
            }

            var input = document.getElementById(target.htmlFor);
            if (input) {
                input.checked = true;
            }

            labels.forEach(function (item) {
                setActiveLabelStyle(item, item === target);
            });
        });

        if (remember) {
            rememberLabel(group, active);
        }
    }

    function refreshActiveLabels() {
        document.querySelectorAll(".tabbed-set").forEach(function (set) {
            var labels = Array.prototype.slice.call(set.querySelectorAll(".tabbed-labels > label"));
            labels.forEach(function (label) {
                var input = document.getElementById(label.htmlFor);
                setActiveLabelStyle(label, Boolean(input && input.checked));
            });
        });
    }

    function bindTabs() {
        document.addEventListener("click", function (event) {
            var label = event.target.closest(".tabbed-labels > label");
            if (!label) {
                return;
            }

            var group = groupForLabel(label.textContent);
            if (!group) {
                return;
            }

            window.requestAnimationFrame(function () {
                activateTabs(group, label.textContent, true);
            });
        });
    }

    function initTabs() {
        var stored = storedLabels();
        GROUPS.forEach(function (group) {
            if (stored[group.key]) {
                activateTabs(group, stored[group.key], false);
            }
        });
        refreshActiveLabels();
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
