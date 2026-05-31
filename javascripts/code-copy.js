(function () {
    "use strict";

    function copyText(text) {
        if (navigator.clipboard && window.isSecureContext) {
            return navigator.clipboard.writeText(text);
        }

        return new Promise(function (resolve, reject) {
            var textarea = document.createElement("textarea");
            textarea.value = text;
            textarea.setAttribute("readonly", "");
            textarea.style.position = "fixed";
            textarea.style.left = "-9999px";
            textarea.style.top = "0";
            document.body.appendChild(textarea);
            textarea.select();

            try {
                if (document.execCommand("copy")) {
                    resolve();
                } else {
                    reject(new Error("copy command failed"));
                }
            } catch (error) {
                reject(error);
            } finally {
                textarea.remove();
            }
        });
    }

    function installButton(block) {
        if (block.querySelector(":scope > .copp-code-copy")) {
            return;
        }

        var code = block.querySelector("pre > code");
        if (!code) {
            return;
        }

        var button = document.createElement("button");
        button.type = "button";
        button.className = "copp-code-copy";
        button.setAttribute("aria-label", "Copy code");
        button.setAttribute("title", "Copy");

        button.addEventListener("click", function () {
            copyText(code.textContent.replace(/\n$/, ""))
                .then(function () {
                    button.classList.add("is-copied");
                    button.setAttribute("aria-label", "Copied");
                    button.setAttribute("title", "Copied");
                    window.setTimeout(function () {
                        button.classList.remove("is-copied");
                        button.setAttribute("aria-label", "Copy code");
                        button.setAttribute("title", "Copy");
                    }, 1400);
                })
                .catch(function () {
                    button.classList.add("is-failed");
                    button.setAttribute("aria-label", "Copy failed");
                    button.setAttribute("title", "Copy failed");
                    window.setTimeout(function () {
                        button.classList.remove("is-failed");
                        button.setAttribute("aria-label", "Copy code");
                        button.setAttribute("title", "Copy");
                    }, 1400);
                });
        });

        block.appendChild(button);
    }

    function initCodeCopy() {
        document.querySelectorAll(".md-typeset .highlight").forEach(installButton);
    }

    if (typeof window.document$ !== "undefined" && window.document$.subscribe) {
        window.document$.subscribe(initCodeCopy);
    }

    if (document.readyState === "loading") {
        document.addEventListener("DOMContentLoaded", initCodeCopy);
    } else {
        initCodeCopy();
    }
})();
