const VERSION = '14.0.0-ZENITH';
const CACHE_NAME = `expear-pro-${VERSION}`;

// Local assets (reliable to cache)
const LOCAL_ASSETS = [
    './',
    './index.html',
    './restoration-ar.html',
    './styles.css',
    './script.js',
    './utils/restoration-engine.js',
    './utils/pose-filter.js',
    './i18n.json',
    './manifest.json',
    './assets/targets.mind'
];

// CDN assets (cached opportunistically — failures don't block install)
const CDN_ASSETS = [
    'https://aframe.io/releases/1.4.2/aframe.min.js',
    'https://cdn.jsdelivr.net/npm/mind-ar@1.2.5/dist/mindar-image-aframe.prod.js',
    'https://fonts.googleapis.com/css2?family=Outfit:wght@300;400;500;600;700;800&display=swap',
    'https://cdnjs.cloudflare.com/ajax/libs/gsap/3.12.2/gsap.min.js',
    'https://cdnjs.cloudflare.com/ajax/libs/howler/2.2.4/howler.min.js'
];

self.addEventListener('install', (event) => {
    self.skipWaiting();
    event.waitUntil(
        caches.open(CACHE_NAME).then(async (cache) => {
            // Local assets must all succeed
            await cache.addAll(LOCAL_ASSETS);
            // CDN assets cached best-effort (don't block install on failure)
            await Promise.allSettled(
                CDN_ASSETS.map(url =>
                    fetch(url, { mode: 'cors' })
                        .then(res => res.ok ? cache.put(url, res) : undefined)
                        .catch(() => {})
                )
            );
        })
    );
});

self.addEventListener('activate', (event) => {
    event.waitUntil(
        caches.keys()
            .then((keys) => Promise.all(
                keys.map((key) => {
                    if (key !== CACHE_NAME) return caches.delete(key);
                })
            ))
            .then(() => self.clients.claim())
    );
});

// Stale-While-Revalidate Strategy
self.addEventListener('fetch', (event) => {
    if (event.request.method !== 'GET') return;

    event.respondWith(
        caches.open(CACHE_NAME).then((cache) => {
            return cache.match(event.request).then((cachedResponse) => {
                const fetchPromise = fetch(event.request)
                    .then((networkResponse) => {
                        if (networkResponse.ok) {
                            cache.put(event.request, networkResponse.clone());
                        }
                        return networkResponse;
                    })
                    .catch(() => cachedResponse);

                return cachedResponse || fetchPromise;
            });
        })
    );
});
