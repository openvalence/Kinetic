// The openvalence.org header's Components menu, a copy of the site's (openvalence.github.io site.js,
// "The Components menu"). Change it there first, then here.
(() => {
  // The Components menu (the header in tools/chrome.html). Mouse: hover opens, leaving closes after
  // a grace, a click on Components navigates. Touch: the first tap opens, the second navigates.
  // Keyboard: the arrow button toggles it, Escape closes it and returns focus to the button, and
  // tabbing out closes it. A click outside closes it.
  for (const m of document.querySelectorAll('.menu')) {
    const btn = m.querySelector('.menu-btn'), link = m.querySelector('.chip');
    let timer = 0, touch = false, hover = false;
    const set = (on, byHover = false) => { clearTimeout(timer); hover = on && byHover; m.classList.toggle('open', on); btn.setAttribute('aria-expanded', String(on)); };
    m.classList.add('live');
    m.addEventListener('pointerenter', (e) => { if (e.pointerType === 'mouse') m.classList.contains('open') ? clearTimeout(timer) : set(true, true); });
    m.addEventListener('pointerleave', (e) => { if (e.pointerType === 'mouse') timer = setTimeout(() => set(false), 300); });
    m.addEventListener('focusout', (e) => { if (!m.contains(e.relatedTarget) && !m.matches(':hover')) set(false); });
    document.addEventListener('keydown', (e) => {
      if (e.key !== 'Escape' || !m.classList.contains('open')) return;
      if (m.querySelector('.menu-panel').contains(document.activeElement)) btn.focus();
      set(false);
    });
    // a click on the arrow of a menu that hover just opened keeps it open
    btn.addEventListener('click', () => set(hover || !m.classList.contains('open')));
    link.addEventListener('pointerdown', (e) => { touch = e.pointerType !== 'mouse'; });
    link.addEventListener('click', (e) => {
      if (touch && !m.classList.contains('open')) { e.preventDefault(); set(true); }
      touch = false;
    });
    document.addEventListener('pointerdown', (e) => { if (!m.contains(e.target)) set(false); });
  }
})();
