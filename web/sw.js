/* ═══════════════════════════════════════════════════════════
   ESP32 Surveillance System — Service Worker (sw.js)
   Enables PWA install + offline shell caching.
   Live WebSocket data is always fetched fresh (network-only).
═══════════════════════════════════════════════════════════ */

const CACHE_NAME   = "esp32-surv-v1";
const CACHE_ASSETS = [
  "./esp32_surveillance_dashboard.html",
  "./manifest.json",
  /* External CDN assets are cached on first fetch */
  "https://cdnjs.cloudflare.com/ajax/libs/nipplejs/0.10.1/nipplejs.min.js",
  "https://fonts.googleapis.com/css2?family=Share+Tech+Mono&family=Orbitron:wght@400;700;900&family=Rajdhani:wght@300;400;600&display=swap"
];

/* ── INSTALL: pre-cache shell assets ── */
self.addEventListener("install", (event) => {
  event.waitUntil(
    caches.open(CACHE_NAME).then((cache) => {
      console.log("[SW] Pre-caching app shell");
      /* Use individual try/catch so one failed CDN request
         doesn't block the whole install */
      return Promise.allSettled(
        CACHE_ASSETS.map(url =>
          cache.add(url).catch(err =>
            console.warn("[SW] Could not cache:", url, err)
          )
        )
      );
    })
  );
  self.skipWaiting(); // activate immediately
});

/* ── ACTIVATE: remove old caches ── */
self.addEventListener("activate", (event) => {
  event.waitUntil(
    caches.keys().then((keys) =>
      Promise.all(
        keys
          .filter(key => key !== CACHE_NAME)
          .map(key => {
            console.log("[SW] Deleting old cache:", key);
            return caches.delete(key);
          })
      )
    )
  );
  self.clients.claim();
});

/* ── FETCH: network-first for WebSocket / API calls,
            cache-first for static assets ── */
self.addEventListener("fetch", (event) => {
  const url = new URL(event.request.url);

  /* WebSocket upgrade requests — let them pass through,
     SW cannot intercept WS, but we skip caching attempts */
  if (event.request.headers.get("upgrade") === "websocket") return;

  /* ws:// or wss:// scheme — skip */
  if (url.protocol === "ws:" || url.protocol === "wss:") return;

  /* For all HTTP(S) requests use cache-first with network fallback */
  event.respondWith(
    caches.match(event.request).then((cachedResponse) => {
      if (cachedResponse) {
        /* Serve from cache; also update cache in background */
        const fetchPromise = fetch(event.request)
          .then(networkResponse => {
            if (networkResponse && networkResponse.status === 200) {
              caches.open(CACHE_NAME).then(cache =>
                cache.put(event.request, networkResponse.clone())
              );
            }
            return networkResponse;
          })
          .catch(() => {/* network unavailable — silently ignore */});
        return cachedResponse; // return cache immediately
      }

      /* Not in cache — fetch from network */
      return fetch(event.request)
        .then(response => {
          /* Cache valid responses */
          if (response && response.status === 200 && response.type !== "opaque") {
            const responseClone = response.clone();
            caches.open(CACHE_NAME).then(cache =>
              cache.put(event.request, responseClone)
            );
          }
          return response;
        })
        .catch(() => {
          /* Offline fallback for navigation requests */
          if (event.request.mode === "navigate") {
            return caches.match("./esp32_surveillance_dashboard.html");
          }
        });
    })
  );
});
