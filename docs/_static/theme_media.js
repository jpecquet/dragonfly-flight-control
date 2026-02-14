function resolvedDocsTheme() {
  const mode = document.body?.dataset?.theme || "auto";
  if (mode === "dark" || mode === "light") {
    return mode;
  }
  return window.matchMedia("(prefers-color-scheme: dark)").matches ? "dark" : "light";
}

function applyThemedMediaSources() {
  const theme = resolvedDocsTheme();
  const videos = document.querySelectorAll("video[data-light-src][data-dark-src]");

  videos.forEach((video) => {
    const src = theme === "dark" ? video.dataset.darkSrc : video.dataset.lightSrc;
    if (!src) {
      return;
    }

    let source = video.querySelector("source");
    if (!source) {
      source = document.createElement("source");
      source.type = "video/mp4";
      video.appendChild(source);
    }

    const current = source.getAttribute("src");
    if (current === src) {
      return;
    }

    source.setAttribute("src", src);
    video.load();
  });
}

function initThemedMediaSources() {
  applyThemedMediaSources();

  const observer = new MutationObserver((mutations) => {
    for (const mutation of mutations) {
      if (mutation.type === "attributes" && mutation.attributeName === "data-theme") {
        applyThemedMediaSources();
        break;
      }
    }
  });

  if (document.body) {
    observer.observe(document.body, { attributes: true, attributeFilter: ["data-theme"] });
  }

  const media = window.matchMedia("(prefers-color-scheme: dark)");
  if (typeof media.addEventListener === "function") {
    media.addEventListener("change", applyThemedMediaSources);
  } else if (typeof media.addListener === "function") {
    media.addListener(applyThemedMediaSources);
  }
}

if (document.readyState === "loading") {
  document.addEventListener("DOMContentLoaded", initThemedMediaSources);
} else {
  initThemedMediaSources();
}
