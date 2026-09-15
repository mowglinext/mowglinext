/*
 * MowgliNext build manual — step engine.
 *
 * Plain browser JS, no build step: GitHub Pages serves docs/ as-is.
 * Content lives in manual-data.js (window.MOWGLI_MANUAL); this file only
 * renders it. See README.md in this folder before editing either.
 *
 * URL contract (mirrors the ArmoredTurtle manual the layout is modelled on):
 *   ?chapter=<chapter id>&step=<1-based index within the FILTERED chapter>&lang=en|fr
 * Per-viewer state (language, hardware profile, done steps) is kept in
 * localStorage under the "mowgli-manual" key; every read/write is guarded so
 * the page still works with storage disabled.
 */
(function () {
  "use strict";

  var STORAGE_KEY = "mowgli-manual";
  var LANGS = ["en", "fr"];

  var UI = {
    en: {
      manual: "Build manual",
      chapters: "Chapters",
      step: "Step",
      of: "of",
      prev: "Previous",
      next: "Next",
      nextChapter: "Next chapter",
      prevChapter: "Previous chapter",
      finished: "You reached the end of the manual.",
      backHome: "Back to mowgli.garden",
      done: "Mark this step as done",
      undone: "Done — click to unmark",
      parts: "You will need",
      profile: "Your build",
      profileHint: "Steps that do not apply to your hardware are hidden.",
      language: "Language",
      print: "Print / PDF",
      menu: "Menu",
      close: "Close",
      skipped: "Hidden for your build",
      progress: "done",
      credit: "Photo",
      chapterOf: "Chapter",
      openImage: "Open full size",
      printTitle: "Printable version",
      wiki: "Wiki",
      github: "GitHub",
    },
    fr: {
      manual: "Guide de montage",
      chapters: "Chapitres",
      step: "Étape",
      of: "sur",
      prev: "Précédent",
      next: "Suivant",
      nextChapter: "Chapitre suivant",
      prevChapter: "Chapitre précédent",
      finished: "Vous êtes arrivé à la fin du guide.",
      backHome: "Retour sur mowgli.garden",
      done: "Marquer cette étape comme faite",
      undone: "Fait — cliquer pour annuler",
      parts: "Il vous faut",
      profile: "Votre montage",
      profileHint: "Les étapes qui ne concernent pas votre matériel sont masquées.",
      language: "Langue",
      print: "Imprimer / PDF",
      menu: "Menu",
      close: "Fermer",
      skipped: "Masqué pour votre montage",
      progress: "faites",
      credit: "Photo",
      chapterOf: "Chapitre",
      openImage: "Ouvrir en grand",
      printTitle: "Version imprimable",
      wiki: "Wiki",
      github: "GitHub",
    },
  };

  // ── Storage ──────────────────────────────────────────────────────────────
  function loadState() {
    try {
      var raw = window.localStorage.getItem(STORAGE_KEY);
      return raw ? JSON.parse(raw) : {};
    } catch (e) {
      return {};
    }
  }

  function saveState(state) {
    try {
      window.localStorage.setItem(STORAGE_KEY, JSON.stringify(state));
    } catch (e) {
      /* storage unavailable: the page keeps working from memory */
    }
  }

  // ── Tiny markdown renderer ───────────────────────────────────────────────
  // Supports exactly what the content uses: paragraphs, ### headings, bullet
  // and numbered lists, task lists, fenced code, tables, blockquotes with
  // GitHub-style callouts (> [!NOTE] / [!TIP] / [!WARNING] / [!DANGER]),
  // and inline **bold**, *italic*, `code`, [text](url).
  function escapeHtml(s) {
    return s
      .replace(/&/g, "&amp;")
      .replace(/</g, "&lt;")
      .replace(/>/g, "&gt;")
      .replace(/"/g, "&quot;");
  }

  function inline(s) {
    var out = escapeHtml(s);
    out = out.replace(/`([^`]+)`/g, function (_, c) {
      return "<code>" + c + "</code>";
    });
    out = out.replace(/\*\*([^*]+)\*\*/g, "<strong>$1</strong>");
    out = out.replace(/(^|[^*])\*([^*\n]+)\*/g, "$1<em>$2</em>");
    out = out.replace(/\[([^\]]+)\]\(([^)\s]+)\)/g, function (_, t, u) {
      var ext = /^https?:\/\//.test(u) ? ' target="_blank" rel="noopener"' : "";
      return '<a href="' + u + '"' + ext + ">" + t + "</a>";
    });
    return out;
  }

  function renderTable(lines) {
    var rows = lines
      .filter(function (l) {
        return !/^\s*\|?\s*:?-{2,}/.test(l);
      })
      .map(function (l) {
        return l
          .replace(/^\s*\|/, "")
          .replace(/\|\s*$/, "")
          .split("|")
          .map(function (c) {
            return c.trim();
          });
      });
    if (!rows.length) return "";
    var html = '<div class="table-wrap"><table><thead><tr>';
    rows[0].forEach(function (c) {
      html += "<th>" + inline(c) + "</th>";
    });
    html += "</tr></thead><tbody>";
    rows.slice(1).forEach(function (r) {
      html += "<tr>";
      r.forEach(function (c) {
        html += "<td>" + inline(c) + "</td>";
      });
      html += "</tr>";
    });
    return html + "</tbody></table></div>";
  }

  function renderList(items, ordered) {
    var tag = ordered ? "ol" : "ul";
    var html = "<" + tag + ">";
    items.forEach(function (it) {
      var m = it.match(/^\[( |x)\]\s+(.*)$/i);
      if (m) {
        html +=
          '<li class="task"><input type="checkbox" disabled' +
          (m[1].toLowerCase() === "x" ? " checked" : "") +
          "> " +
          inline(m[2]) +
          "</li>";
      } else {
        html += "<li>" + inline(it) + "</li>";
      }
    });
    return html + "</" + tag + ">";
  }

  function markdown(src) {
    if (!src) return "";
    var lines = src.replace(/\r/g, "").split("\n");
    var html = "";
    var i = 0;
    while (i < lines.length) {
      var line = lines[i];
      if (!line.trim()) {
        i++;
        continue;
      }
      if (/^```/.test(line)) {
        var code = [];
        i++;
        while (i < lines.length && !/^```/.test(lines[i])) {
          code.push(lines[i]);
          i++;
        }
        i++;
        html += "<pre><code>" + escapeHtml(code.join("\n")) + "</code></pre>";
        continue;
      }
      var h = line.match(/^(#{1,4})\s+(.*)$/);
      if (h) {
        var lvl = Math.min(h[1].length + 2, 5);
        html += "<h" + lvl + ">" + inline(h[2]) + "</h" + lvl + ">";
        i++;
        continue;
      }
      if (/^\s*\|/.test(line)) {
        var tbl = [];
        while (i < lines.length && /^\s*\|/.test(lines[i])) {
          tbl.push(lines[i]);
          i++;
        }
        html += renderTable(tbl);
        continue;
      }
      if (/^>/.test(line)) {
        var quote = [];
        while (i < lines.length && /^>/.test(lines[i])) {
          quote.push(lines[i].replace(/^>\s?/, ""));
          i++;
        }
        var kind = "note";
        var first = quote[0].match(/^\[!(NOTE|TIP|WARNING|DANGER)\]\s*(.*)$/i);
        var title = "";
        if (first) {
          kind = first[1].toLowerCase();
          title = first[2];
          quote.shift();
        }
        html +=
          '<div class="callout callout-' +
          kind +
          '">' +
          (title ? '<div class="callout-title">' + inline(title) + "</div>" : "") +
          markdown(quote.join("\n")) +
          "</div>";
        continue;
      }
      if (/^\s*[-*]\s+/.test(line)) {
        var ul = [];
        while (i < lines.length && /^\s*[-*]\s+/.test(lines[i])) {
          ul.push(lines[i].replace(/^\s*[-*]\s+/, ""));
          i++;
        }
        html += renderList(ul, false);
        continue;
      }
      if (/^\s*\d+[.)]\s+/.test(line)) {
        var ol = [];
        while (i < lines.length && /^\s*\d+[.)]\s+/.test(lines[i])) {
          ol.push(lines[i].replace(/^\s*\d+[.)]\s+/, ""));
          i++;
        }
        html += renderList(ol, true);
        continue;
      }
      var para = [];
      while (
        i < lines.length &&
        lines[i].trim() &&
        !/^(```|#{1,4}\s|\s*\||>|\s*[-*]\s+|\s*\d+[.)]\s+)/.test(lines[i])
      ) {
        para.push(lines[i]);
        i++;
      }
      html += "<p>" + inline(para.join(" ")) + "</p>";
    }
    return html;
  }

  // ── Model ────────────────────────────────────────────────────────────────
  var data = window.MOWGLI_MANUAL;
  if (!data) {
    document.body.innerHTML =
      '<p style="padding:24px">manual-data.js failed to load. Open the browser console for details.</p>';
    return;
  }

  var state = loadState();
  var lang = pickLang();
  var profile = pickProfile();
  var done = state.done || {};

  function t(key) {
    return (UI[lang] && UI[lang][key]) || UI.en[key] || key;
  }

  function tx(obj) {
    if (obj == null) return "";
    if (typeof obj === "string") return obj;
    return obj[lang] || obj.en || "";
  }

  function pickLang() {
    var q = new URLSearchParams(window.location.search).get("lang");
    if (q && LANGS.indexOf(q) >= 0) return q;
    if (state.lang && LANGS.indexOf(state.lang) >= 0) return state.lang;
    var nav = (navigator.language || "en").slice(0, 2).toLowerCase();
    return LANGS.indexOf(nav) >= 0 ? nav : "en";
  }

  function pickProfile() {
    var p = {};
    Object.keys(data.profile).forEach(function (k) {
      var opts = data.profile[k].options.map(function (o) {
        return o.id;
      });
      var saved = state.profile && state.profile[k];
      p[k] = opts.indexOf(saved) >= 0 ? saved : opts[0];
    });
    return p;
  }

  function stepApplies(step) {
    if (!step.when) return true;
    return Object.keys(step.when).every(function (k) {
      var allowed = step.when[k];
      return allowed.indexOf(profile[k]) >= 0;
    });
  }

  function visibleSteps(chapter) {
    return chapter.steps.filter(stepApplies);
  }

  function chapterIndex(id) {
    for (var i = 0; i < data.chapters.length; i++) {
      if (data.chapters[i].id === id) return i;
    }
    return 0;
  }

  // ── Location ─────────────────────────────────────────────────────────────
  var current = { chapter: 0, step: 0 };

  function readLocation() {
    var q = new URLSearchParams(window.location.search);
    var ch = chapterIndex(q.get("chapter") || data.chapters[0].id);
    var steps = visibleSteps(data.chapters[ch]);
    var st = parseInt(q.get("step") || "1", 10);
    if (isNaN(st) || st < 1) st = 1;
    if (st > steps.length) st = steps.length;
    current = { chapter: ch, step: st - 1 };
  }

  function writeLocation(push) {
    var q = new URLSearchParams();
    q.set("chapter", data.chapters[current.chapter].id);
    q.set("step", String(current.step + 1));
    q.set("lang", lang);
    var url = window.location.pathname + "?" + q.toString();
    if (push) window.history.pushState(null, "", url);
    else window.history.replaceState(null, "", url);
  }

  function go(chapter, step, push) {
    var steps = visibleSteps(data.chapters[chapter]);
    if (!steps.length) return;
    current = {
      chapter: chapter,
      step: Math.max(0, Math.min(step, steps.length - 1)),
    };
    writeLocation(push !== false);
    render();
    window.scrollTo(0, 0);
    var pane = document.getElementById("instructions");
    if (pane) pane.scrollTop = 0;
  }

  function next() {
    var steps = visibleSteps(data.chapters[current.chapter]);
    if (current.step < steps.length - 1) return go(current.chapter, current.step + 1);
    var ch = current.chapter + 1;
    while (ch < data.chapters.length && !visibleSteps(data.chapters[ch]).length) ch++;
    if (ch < data.chapters.length) go(ch, 0);
  }

  function prev() {
    if (current.step > 0) return go(current.chapter, current.step - 1);
    var ch = current.chapter - 1;
    while (ch >= 0 && !visibleSteps(data.chapters[ch]).length) ch--;
    if (ch >= 0) go(ch, visibleSteps(data.chapters[ch]).length - 1);
  }

  // ── Rendering ────────────────────────────────────────────────────────────
  function el(id) {
    return document.getElementById(id);
  }

  function chapterProgress(chapter) {
    var steps = visibleSteps(chapter);
    var n = steps.filter(function (s) {
      return done[s.id];
    }).length;
    return { done: n, total: steps.length };
  }

  function renderSidebar() {
    var list = el("chapter-list");
    list.innerHTML = "";
    data.chapters.forEach(function (ch, idx) {
      var steps = visibleSteps(ch);
      if (!steps.length) return;
      var p = chapterProgress(ch);
      var li = document.createElement("li");
      var a = document.createElement("a");
      a.href = "?chapter=" + ch.id + "&step=1&lang=" + lang;
      a.className = "chapter-link" + (idx === current.chapter ? " active" : "");
      a.innerHTML =
        '<span class="chapter-icon" aria-hidden="true">' +
        ch.icon +
        "</span>" +
        '<span class="chapter-name">' +
        escapeHtml(tx(ch.title)) +
        "</span>" +
        '<span class="chapter-progress' +
        (p.done === p.total ? " complete" : "") +
        '">' +
        p.done +
        "/" +
        p.total +
        "</span>";
      a.addEventListener("click", function (ev) {
        ev.preventDefault();
        closeMenu();
        go(idx, 0);
      });
      li.appendChild(a);
      if (idx === current.chapter) {
        var sub = document.createElement("ol");
        sub.className = "step-list";
        steps.forEach(function (s, sIdx) {
          var sl = document.createElement("li");
          var sa = document.createElement("a");
          sa.href = "?chapter=" + ch.id + "&step=" + (sIdx + 1) + "&lang=" + lang;
          sa.className =
            "step-link" + (sIdx === current.step ? " active" : "") + (done[s.id] ? " done" : "");
          sa.textContent = tx(s.title);
          sa.addEventListener("click", function (ev) {
            ev.preventDefault();
            closeMenu();
            go(idx, sIdx);
          });
          sl.appendChild(sa);
          sub.appendChild(sl);
        });
        li.appendChild(sub);
      }
      list.appendChild(li);
    });
  }

  function renderProfile() {
    var box = el("profile-form");
    box.innerHTML = "";
    Object.keys(data.profile).forEach(function (key) {
      var group = data.profile[key];
      var wrap = document.createElement("div");
      wrap.className = "profile-group";
      var label = document.createElement("div");
      label.className = "profile-label";
      label.textContent = tx(group.label);
      wrap.appendChild(label);
      var opts = document.createElement("div");
      opts.className = "profile-options";
      opts.setAttribute("role", "radiogroup");
      opts.setAttribute("aria-label", tx(group.label));
      group.options.forEach(function (o) {
        var b = document.createElement("button");
        b.type = "button";
        b.className = "profile-option" + (profile[key] === o.id ? " active" : "");
        b.setAttribute("role", "radio");
        b.setAttribute("aria-checked", profile[key] === o.id ? "true" : "false");
        b.textContent = tx(o.label);
        b.addEventListener("click", function () {
          profile[key] = o.id;
          state.profile = profile;
          saveState(state);
          // The filtered step list changed: clamp the position and re-render.
          go(current.chapter, current.step, false);
        });
        opts.appendChild(b);
      });
      wrap.appendChild(opts);
      box.appendChild(wrap);
    });
  }

  function renderMedia(chapter, step) {
    var pane = el("media");
    var media = step.media || chapter.media;
    if (!media) {
      pane.innerHTML =
        '<div class="media-placeholder"><span class="media-emoji" aria-hidden="true">' +
        chapter.icon +
        '</span><span class="media-chapter">' +
        escapeHtml(tx(chapter.title)) +
        "</span></div>";
      return;
    }
    var html = "";
    if (media.type === "video") {
      html =
        '<video controls playsinline preload="metadata" src="' +
        media.src +
        '" poster="' +
        (media.poster || "") +
        '"></video>';
    } else {
      html =
        '<a class="media-link" href="' +
        media.src +
        '" target="_blank" rel="noopener" title="' +
        escapeHtml(t("openImage")) +
        '"><img src="' +
        media.src +
        '" alt="' +
        escapeHtml(tx(media.alt)) +
        '" loading="eager"></a>';
    }
    if (media.caption || media.credit) {
      html += '<div class="media-caption">' + inline(tx(media.caption) || "");
      if (media.credit) {
        html +=
          ' <span class="media-credit">' +
          t("credit") +
          ": " +
          inline(tx(media.credit)) +
          "</span>";
      }
      html += "</div>";
    }
    pane.innerHTML = html;
  }

  function renderStep() {
    var chapter = data.chapters[current.chapter];
    var steps = visibleSteps(chapter);
    var step = steps[current.step];
    if (!step) return;

    el("chapter-kicker").textContent =
      t("chapterOf") + " " + (current.chapter + 1) + " — " + tx(chapter.title);
    el("step-title").textContent = tx(step.title);
    el("step-body").innerHTML = markdown(tx(step.body));

    var partsBox = el("parts");
    if (step.parts && step.parts.length) {
      var lis = step.parts
        .map(function (p) {
          var qty = p.qty ? '<span class="part-qty">' + escapeHtml(String(p.qty)) + "×</span> " : "";
          return "<li>" + qty + inline(tx(p.name || p)) + "</li>";
        })
        .join("");
      partsBox.innerHTML = "<h3>" + t("parts") + "</h3><ul>" + lis + "</ul>";
      partsBox.hidden = false;
    } else {
      partsBox.hidden = true;
      partsBox.innerHTML = "";
    }

    var doneBtn = el("done-toggle");
    doneBtn.textContent = done[step.id] ? "✓ " + t("undone") : t("done");
    doneBtn.className = "done-toggle" + (done[step.id] ? " is-done" : "");
    doneBtn.onclick = function () {
      if (done[step.id]) delete done[step.id];
      else done[step.id] = true;
      state.done = done;
      saveState(state);
      render();
    };

    renderMedia(chapter, step);

    el("step-input").value = String(current.step + 1);
    el("step-input").max = String(steps.length);
    el("step-total").textContent = "/ " + steps.length;

    var lastStep = current.step === steps.length - 1;
    var lastChapter = current.chapter === data.chapters.length - 1;
    el("next-step").disabled = lastStep && lastChapter;
    el("prev-step").disabled = current.chapter === 0 && current.step === 0;

    var tail = el("chapter-tail");
    if (lastStep && !lastChapter) {
      var nextCh = data.chapters[current.chapter + 1];
      tail.innerHTML =
        '<button type="button" class="btn btn-primary" id="tail-next">' +
        t("nextChapter") +
        " → " +
        escapeHtml(tx(nextCh.title)) +
        "</button>";
      el("tail-next").addEventListener("click", next);
      tail.hidden = false;
    } else if (lastStep && lastChapter) {
      tail.innerHTML =
        '<p class="finished">' +
        t("finished") +
        '</p><a class="btn btn-secondary" href="../">' +
        t("backHome") +
        "</a>";
      tail.hidden = false;
    } else {
      tail.hidden = true;
      tail.innerHTML = "";
    }

    document.title = tx(step.title) + " — " + tx(data.title);
  }

  function renderChrome() {
    document.documentElement.lang = lang;
    el("manual-title").textContent = tx(data.title);
    el("chapters-heading").textContent = t("chapters");
    el("profile-heading").textContent = t("profile");
    el("profile-hint").textContent = t("profileHint");
    el("lang-heading").textContent = t("language");
    el("prev-step").setAttribute("aria-label", t("prev"));
    el("next-step").setAttribute("aria-label", t("next"));
    el("menu-button").setAttribute("aria-label", t("menu"));
    el("print-link").textContent = t("print");
    el("print-link").href = "print.html?lang=" + lang;
    el("wiki-link").textContent = t("wiki");
    el("github-link").textContent = t("github");
    LANGS.forEach(function (l) {
      var b = el("lang-" + l);
      if (!b) return;
      b.className = "lang-option" + (l === lang ? " active" : "");
      b.setAttribute("aria-pressed", l === lang ? "true" : "false");
    });
  }

  function render() {
    renderChrome();
    renderSidebar();
    renderProfile();
    renderStep();
  }

  // ── Menu (mobile drawer) ─────────────────────────────────────────────────
  function openMenu() {
    document.body.classList.add("menu-open");
    el("menu-button").setAttribute("aria-expanded", "true");
  }
  function closeMenu() {
    document.body.classList.remove("menu-open");
    el("menu-button").setAttribute("aria-expanded", "false");
  }

  // ── Wire up ──────────────────────────────────────────────────────────────
  function init() {
    readLocation();
    writeLocation(false);

    el("prev-step").addEventListener("click", prev);
    el("next-step").addEventListener("click", next);
    el("step-input").addEventListener("change", function () {
      var n = parseInt(this.value, 10);
      if (!isNaN(n)) go(current.chapter, n - 1);
    });
    el("menu-button").addEventListener("click", function () {
      if (document.body.classList.contains("menu-open")) closeMenu();
      else openMenu();
    });
    el("menu-backdrop").addEventListener("click", closeMenu);
    LANGS.forEach(function (l) {
      var b = el("lang-" + l);
      if (!b) return;
      b.addEventListener("click", function () {
        lang = l;
        state.lang = l;
        saveState(state);
        writeLocation(false);
        render();
      });
    });
    document.addEventListener("keydown", function (ev) {
      var tag = (ev.target && ev.target.tagName) || "";
      if (tag === "INPUT" || tag === "TEXTAREA") return;
      if (ev.key === "ArrowRight") next();
      else if (ev.key === "ArrowLeft") prev();
      else if (ev.key === "Escape") closeMenu();
    });
    window.addEventListener("popstate", function () {
      readLocation();
      render();
    });

    render();
  }

  // Expose the renderer for print.html, which reuses the markdown + i18n
  // helpers to lay the whole manual out linearly.
  window.MowgliManual = {
    markdown: markdown,
    inline: inline,
    escapeHtml: escapeHtml,
    ui: UI,
    langs: LANGS,
  };

  if (document.getElementById("chapter-list")) {
    if (document.readyState === "loading") document.addEventListener("DOMContentLoaded", init);
    else init();
  }
})();
