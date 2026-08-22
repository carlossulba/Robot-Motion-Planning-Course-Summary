// Shared chrome: theme persistence (light/dark toggle).
(function () {
  var THEME_KEY = "rmp-theme";
  var root = document.documentElement;

  function applyTheme(t) {
    if (t === "dark" || t === "light") {
      root.setAttribute("data-theme", t);
    } else {
      root.removeAttribute("data-theme");
    }
  }

  var saved = localStorage.getItem(THEME_KEY);
  applyTheme(saved);

  function currentEffective() {
    var attr = root.getAttribute("data-theme");
    if (attr) return attr;
    return window.matchMedia("(prefers-color-scheme: dark)").matches ? "dark" : "light";
  }

  document.addEventListener("DOMContentLoaded", function () {
    var btn = document.querySelector(".theme-toggle");
    if (btn) {
      btn.textContent = currentEffective() === "dark" ? "☀" : "☾";
      btn.addEventListener("click", function () {
        var next = currentEffective() === "dark" ? "light" : "dark";
        applyTheme(next);
        localStorage.setItem(THEME_KEY, next);
        btn.textContent = next === "dark" ? "☀" : "☾";
      });
    }
  });
})();
