// Native links preserve immediate navigation, keyboard activation and new tabs.
const currentPage = window.location.pathname.split('/').pop() || 'index.html';
const nav = document.querySelector('.nav');
const links = nav ? Array.from(nav.querySelectorAll('a')) : [];
links.forEach((link, index) => {
  if (link.getAttribute('href') === currentPage) {
    link.classList.add('is-active');
    link.setAttribute('aria-current', 'page');
    nav.style.setProperty('--active-index', index);
  }
});
if (nav) nav.style.setProperty('--page-count', links.length);

const progress = document.querySelector('.scroll-progress');
let scheduled = false;
function updateProgress() {
  if (progress) {
    const range = document.documentElement.scrollHeight - window.innerHeight;
    progress.style.transform = `scaleX(${range > 0 ? Math.min(1, Math.max(0, window.scrollY / range)) : 0})`;
  }
  scheduled = false;
}
window.addEventListener('scroll', () => {
  if (!scheduled) {
    scheduled = true;
    requestAnimationFrame(updateProgress);
  }
}, { passive: true });
window.addEventListener('resize', updateProgress);
window.addEventListener('load', updateProgress);
updateProgress();
