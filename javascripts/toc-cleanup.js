(function () {
    "use strict";

    var hasCjk = /[\u3400-\u4dbf\u4e00-\u9fff\uf900-\ufaff]/;

    function stripEnglishParentheses(text) {
        if (!hasCjk.test(text)) {
            return text;
        }
        return text.replace(/\s*\((?=[^)]*[A-Za-z])[^)]*\)/g, "");
    }

    function isChinesePage() {
        var heading = document.querySelector(".md-content h1");
        return (
            (heading && hasCjk.test(heading.textContent)) ||
            /(?:^|\/)index-zh(?:\.html)?$/.test(window.location.pathname)
        );
    }

    function cleanupToc(root) {
        (root || document).querySelectorAll("[data-md-component='toc'] .md-nav__link").forEach(function (link) {
            var original = link.getAttribute("data-copp-toc-original") || link.textContent;
            link.setAttribute("data-copp-toc-original", original);
            link.textContent = stripEnglishParentheses(original);
        });
    }

    function removeScrollFix(root) {
        (root || document).querySelectorAll("[data-md-scrollfix]").forEach(function (element) {
            element.removeAttribute("data-md-scrollfix");
        });
    }

    function installSidebarWheelScroll() {
        var scroller = document.querySelector(".md-sidebar--primary .md-sidebar__scrollwrap");
        if (!scroller || scroller.getAttribute("data-copp-wheel-scroll") === "1") {
            return;
        }

        scroller.setAttribute("data-copp-wheel-scroll", "1");
        scroller.addEventListener(
            "wheel",
            function (event) {
                var maxScroll = scroller.scrollHeight - scroller.clientHeight;
                if (maxScroll <= 0 || Math.abs(event.deltaY) <= Math.abs(event.deltaX)) {
                    return;
                }

                var before = scroller.scrollTop;
                scroller.scrollTop = Math.max(0, Math.min(maxScroll, before + event.deltaY));

                if (scroller.scrollTop !== before) {
                    event.preventDefault();
                    event.stopImmediatePropagation();
                }
            },
            { capture: true, passive: false }
        );
    }

    function moveTocToPrimarySidebar() {
        document.querySelectorAll(".copp-sidebar-toc").forEach(function (existing) {
            existing.remove();
        });

        var primaryList = document.querySelector(".md-sidebar--primary .md-nav--primary > .md-nav__list");
        var secondarySidebar = document.querySelector(".md-sidebar--secondary");
        var tocList =
            document.querySelector(".md-sidebar--secondary [data-md-component='toc']") ||
            document.querySelector(".md-sidebar--primary .md-nav__item--active > .md-nav--secondary [data-md-component='toc']");

        if (secondarySidebar) {
            secondarySidebar.hidden = true;
            secondarySidebar.setAttribute("aria-hidden", "true");
        }

        if (!primaryList || !tocList) {
            cleanupToc();
            installSidebarWheelScroll();
            return;
        }

        var item = document.createElement("li");
        item.className = "md-nav__item copp-sidebar-toc";

        var title = document.createElement("span");
        title.className = "copp-sidebar-toc__title";
        title.textContent = isChinesePage() ? "目录" : "Contents";
        item.appendChild(title);

        var nav = document.createElement("nav");
        nav.className = "md-nav md-nav--secondary copp-sidebar-toc__nav";
        nav.setAttribute("aria-label", title.textContent);
        nav.appendChild(tocList.cloneNode(true));
        removeScrollFix(nav);
        item.appendChild(nav);

        primaryList.appendChild(item);
        cleanupToc(item);
        installSidebarWheelScroll();
    }

    if (typeof window.document$ !== "undefined" && window.document$.subscribe) {
        window.document$.subscribe(moveTocToPrimarySidebar);
    }

    if (document.readyState === "loading") {
        document.addEventListener("DOMContentLoaded", moveTocToPrimarySidebar);
    } else {
        moveTocToPrimarySidebar();
    }
})();
