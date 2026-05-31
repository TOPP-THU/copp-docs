(function () {
  "use strict";

  var STORAGE_KEY = "copp.docs.language";
  var DEFAULT_LANGS = ["rust", "c"];
  var LABELS = {
    rust: "Rust",
    c: "C"
  };

  function normalizeLang(value) {
    return String(value || "").trim().toLowerCase();
  }

  function labelFor(lang) {
    return LABELS[lang] || lang;
  }

  function parseLanguages(value) {
    var raw = String(value || "");
    var parts = raw.indexOf(":") >= 0 ? raw.split(":").slice(1).join(":") : "";
    var langs = parts.split(",").map(normalizeLang).filter(Boolean);
    return langs.length ? langs : DEFAULT_LANGS.slice();
  }

  function markerBlock(marker) {
    var parent = marker.parentElement;
    if (
      parent &&
      parent !== document.body &&
      parent.childElementCount === 1 &&
      parent.textContent.trim() === ""
    ) {
      return parent;
    }
    return marker;
  }

  function markerValue(marker) {
    return marker.getAttribute("data-copp-tabs") || "";
  }

  function isEndMarkerNode(node) {
    if (!node || node.nodeType !== 1) {
      return null;
    }
    if (markerValue(node).indexOf("end") === 0) {
      return node;
    }
    return node.querySelector('[data-copp-tabs^="end"]');
  }

  function headingLanguage(node, langs) {
    if (!node || node.nodeType !== 1 || !/^H[1-6]$/.test(node.tagName)) {
      return "";
    }
    var text = node.textContent.trim().toLowerCase();
    for (var i = 0; i < langs.length; i += 1) {
      var lang = langs[i];
      if (text === lang || text === labelFor(lang).toLowerCase()) {
        return lang;
      }
    }
    return "";
  }

  function createButton(lang, index, groupId) {
    var button = document.createElement("button");
    button.type = "button";
    button.className = "copp-tabs__button";
    button.setAttribute("role", "tab");
    button.setAttribute("aria-controls", groupId + "-panel-" + lang);
    button.setAttribute("id", groupId + "-tab-" + lang);
    button.setAttribute("data-copp-tab-button", lang);
    button.textContent = labelFor(lang);
    if (index === 0) {
      button.classList.add("is-active");
      button.setAttribute("aria-selected", "true");
    } else {
      button.setAttribute("aria-selected", "false");
    }
    return button;
  }

  function createTabs(nodes, langs, groupIndex) {
    var panels = {};
    var order = [];
    var current = "";

    langs.forEach(function (lang) {
      panels[lang] = [];
    });

    nodes.forEach(function (node) {
      var lang = headingLanguage(node, langs);
      if (lang) {
        current = lang;
        if (order.indexOf(lang) < 0) {
          order.push(lang);
        }
        node.remove();
        return;
      }
      if (!current && node.textContent && node.textContent.trim()) {
        current = langs[0];
        if (order.indexOf(current) < 0) {
          order.push(current);
        }
      }
      if (current) {
        panels[current].push(node);
      }
    });

    order = order.filter(function (lang) {
      return panels[lang] && panels[lang].length;
    });

    if (!order.length) {
      return null;
    }

    var groupId = "copp-tabs-" + groupIndex;
    var section = document.createElement("section");
    section.className = "copp-tabs";
    section.setAttribute("data-copp-tabs-widget", "");

    var bar = document.createElement("div");
    bar.className = "copp-tabs__bar";
    bar.setAttribute("role", "tablist");
    order.forEach(function (lang, index) {
      bar.appendChild(createButton(lang, index, groupId));
    });
    section.appendChild(bar);

    order.forEach(function (lang, index) {
      var panel = document.createElement("div");
      panel.className = "copp-tabs__panel";
      panel.setAttribute("role", "tabpanel");
      panel.setAttribute("id", groupId + "-panel-" + lang);
      panel.setAttribute("aria-labelledby", groupId + "-tab-" + lang);
      panel.setAttribute("data-copp-panel", lang);
      if (index !== 0) {
        panel.hidden = true;
      }
      panels[lang].forEach(function (node) {
        panel.appendChild(node);
      });
      section.appendChild(panel);
    });

    return section;
  }

  function buildTabsFromMarkers() {
    var groupIndex = document.querySelectorAll(".copp-tabs").length;

    while (true) {
      var start = document.querySelector('[data-copp-tabs^="start"]');
      if (!start) {
        break;
      }

      var langs = parseLanguages(markerValue(start));
      var startBlock = markerBlock(start);
      var parent = startBlock.parentNode;
      var nodes = [];
      var endBlock = null;
      var cursor = startBlock.nextSibling;

      while (cursor) {
        var next = cursor.nextSibling;
        var endMarker = isEndMarkerNode(cursor);
        if (endMarker) {
          endBlock = markerBlock(endMarker);
          break;
        }
        nodes.push(cursor);
        cursor = next;
      }

      if (!parent || !endBlock) {
        start.remove();
        continue;
      }

      var tabs = createTabs(nodes, langs, groupIndex);
      groupIndex += 1;
      if (tabs) {
        parent.insertBefore(tabs, startBlock);
      }
      startBlock.remove();
      endBlock.remove();
    }
  }

  function availableLanguages() {
    var seen = {};
    var langs = [];
    document.querySelectorAll("[data-copp-panel]").forEach(function (panel) {
      var lang = normalizeLang(panel.getAttribute("data-copp-panel"));
      if (lang && !seen[lang]) {
        seen[lang] = true;
        langs.push(lang);
      }
    });
    return DEFAULT_LANGS.concat(langs).filter(function (lang, index, all) {
      return lang && all.indexOf(lang) === index && seen[lang];
    });
  }

  function initialLanguage(langs) {
    var params = new URLSearchParams(window.location.search);
    var fromUrl = normalizeLang(params.get("lang") || params.get("copp-lang"));
    var fromStorage = "";
    try {
      fromStorage = normalizeLang(window.localStorage.getItem(STORAGE_KEY));
    } catch (error) {
      fromStorage = "";
    }
    if (langs.indexOf(fromUrl) >= 0) {
      return fromUrl;
    }
    if (langs.indexOf(fromStorage) >= 0) {
      return fromStorage;
    }
    return langs.indexOf("rust") >= 0 ? "rust" : langs[0];
  }

  function insertGlobalSwitch(langs) {
    if (document.querySelector(".copp-lang-switch") || langs.length < 2) {
      return;
    }

    var switcher = document.createElement("div");
    switcher.className = "copp-lang-switch";
    switcher.setAttribute("role", "group");
    switcher.setAttribute("aria-label", "代码语言");

    var label = document.createElement("span");
    label.className = "copp-lang-switch__label";
    label.textContent = "代码语言";
    switcher.appendChild(label);

    var buttons = document.createElement("div");
    buttons.className = "copp-lang-switch__buttons";
    langs.forEach(function (lang) {
      var button = document.createElement("button");
      button.type = "button";
      button.className = "copp-lang-switch__button";
      button.setAttribute("data-copp-global-tab", lang);
      button.textContent = labelFor(lang);
      buttons.appendChild(button);
    });
    switcher.appendChild(buttons);

    var root = document.querySelector("#write") || document.body;
    var hero = root.querySelector(".copp-hero");
    var h1 = root.querySelector("h1");
    if (hero && hero.parentNode) {
      hero.parentNode.insertBefore(switcher, hero.nextSibling);
    } else if (h1 && h1.parentNode) {
      h1.parentNode.insertBefore(switcher, h1.nextSibling);
    } else {
      root.insertBefore(switcher, root.firstChild);
    }
  }

  function setLanguage(lang) {
    var active = normalizeLang(lang);
    if (!active) {
      return;
    }

    try {
      window.localStorage.setItem(STORAGE_KEY, active);
    } catch (error) {
      // Ignore storage errors in private or restricted browsing contexts.
    }

    document.documentElement.setAttribute("data-copp-lang", active);

    document.querySelectorAll(".copp-tabs").forEach(function (tabs) {
      var panels = Array.prototype.slice.call(tabs.querySelectorAll("[data-copp-panel]"));
      var target = panels.find(function (panel) {
        return normalizeLang(panel.getAttribute("data-copp-panel")) === active;
      }) || panels[0];

      panels.forEach(function (panel) {
        panel.hidden = panel !== target;
      });

      tabs.querySelectorAll("[data-copp-tab-button]").forEach(function (button) {
        var isActive = normalizeLang(button.getAttribute("data-copp-tab-button")) === normalizeLang(target.getAttribute("data-copp-panel"));
        button.classList.toggle("is-active", isActive);
        button.setAttribute("aria-selected", isActive ? "true" : "false");
      });
    });

    document.querySelectorAll("[data-copp-global-tab]").forEach(function (button) {
      button.classList.toggle(
        "is-active",
        normalizeLang(button.getAttribute("data-copp-global-tab")) === active
      );
    });

    if (window.MathJax && typeof window.MathJax.typesetPromise === "function") {
      window.MathJax.typesetPromise(
        Array.prototype.slice.call(document.querySelectorAll(".copp-tabs__panel:not([hidden])"))
      ).catch(function () {});
    }
  }

  function bindEvents() {
    document.addEventListener("click", function (event) {
      var button = event.target.closest("[data-copp-tab-button], [data-copp-global-tab]");
      if (!button) {
        return;
      }
      var lang = button.getAttribute("data-copp-tab-button") || button.getAttribute("data-copp-global-tab");
      setLanguage(lang);
    });
  }

  function init() {
    buildTabsFromMarkers();
    var langs = availableLanguages();
    if (!langs.length) {
      return;
    }
    insertGlobalSwitch(langs);
    bindEvents();
    setLanguage(initialLanguage(langs));
  }

  if (document.readyState === "loading") {
    document.addEventListener("DOMContentLoaded", init);
  } else {
    init();
  }
})();
