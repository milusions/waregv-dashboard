const STORAGE_KEY = 'Rover host';
const STORAGE_EXPIRY_KEY = 'Rover host expires';
const LOGIN_TTL_MS = 24 * 60 * 60 * 1000;

function isValidIPv4(value) {
  if (!value) return false;
  const input = String(value).trim();
  if (input === 'localhost') return true;
  const parts = input.split('.');
  if (parts.length !== 4) return false;
  return parts.every((part) => /^\d+$/.test(part) && Number(part) >= 0 && Number(part) <= 255);
}

function readHost() {
  try {
    const value = localStorage.getItem(STORAGE_KEY);
    return value ? String(value).trim() : '';
  } catch (error) {
    return '';
  }
}

function readExpiry() {
  try {
    const value = Number(localStorage.getItem(STORAGE_EXPIRY_KEY) || 0);
    return Number.isFinite(value) ? value : 0;
  } catch (error) {
    return 0;
  }
}

function saveHost(host) {
  const value = String(host || '').trim();
  if (!value) return;
  try {
    localStorage.setItem(STORAGE_KEY, value);
    localStorage.setItem(STORAGE_EXPIRY_KEY, String(Date.now() + LOGIN_TTL_MS));
  } catch (error) {
    // Ignore storage errors in private browsing mode.
  }
}

function isValidStoredSession() {
  const host = readHost();
  const expiry = readExpiry();
  return Boolean(host && isValidIPv4(host) && Date.now() < expiry);
}

function redirectToDashboard() {
  window.location.href = 'dashboard.html';
}

function validateHostAndPassword(username, password) {
  const providedUser = String(username || '').trim().toLowerCase();
  const host = String(password || '').trim();

  if (providedUser !== 'waregv') {
    return Promise.reject(new Error('Username must be waregv.'));
  }

  if (!isValidIPv4(host)) {
    return Promise.reject(new Error('Password must be a valid Ware GV IP address or localhost.'));
  }

  const target = host === 'localhost' ? 'http://localhost:8000/' : `http://${host}:8000/`;

  return fetch(target, { cache: 'no-store' })
    .then((response) => {
      if (response && (response.ok || response.type === 'opaque' || (response.status >= 200 && response.status < 600))) {
        return true;
      }
      return true;
    })
    .catch(() => {
      throw new Error('The rover did not respond at that IP. Please check the connection and try again.');
    });
}

function bindLoginForm() {
  const form = document.getElementById('login-form');
  const username = document.getElementById('username');
  const password = document.getElementById('password');
  const errorBox = document.getElementById('login-error');
  const submitBtn = document.getElementById('login-submit');

  if (!form || !username || !password || !errorBox || !submitBtn) return;

  username.value = 'waregv';
  username.readOnly = true;

  const storedHost = readHost();
  if (storedHost && isValidIPv4(storedHost)) {
    password.value = storedHost;
  }

  form.addEventListener('submit', (event) => {
    event.preventDefault();
    errorBox.textContent = '';
    submitBtn.disabled = true;
    submitBtn.textContent = 'Checking…';

    validateHostAndPassword(username.value, password.value)
      .then(() => {
        saveHost(password.value);
        redirectToDashboard();
      })
      .catch((error) => {
        errorBox.textContent = error.message || 'Login failed.';
      })
      .finally(() => {
        submitBtn.disabled = false;
        submitBtn.textContent = 'Sign in';
      });
  });
}

window.addEventListener('DOMContentLoaded', () => {
  if (isValidStoredSession()) {
    redirectToDashboard();
    return;
  }
  bindLoginForm();
});
