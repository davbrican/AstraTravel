FROM nginx:stable-alpine
COPY web/nginx.conf /etc/nginx/conf.d/default.conf
COPY web/index.html web/styles.css web/app.js web/renderer.js web/physics.js web/worker.js web/favicon.svg /usr/share/nginx/html/
EXPOSE 80
HEALTHCHECK --interval=30s --timeout=3s --start-period=5s CMD wget -q -O /dev/null http://127.0.0.1/ || exit 1
