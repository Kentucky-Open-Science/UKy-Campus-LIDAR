# Lexington Digital Twin — server image.
#
# Serves the viewer + shared-world API + live transit/cameras, and (via --bootstrap)
# downloads & builds the WHOLE city on first start: ~114k buildings, roads, traffic
# lights, intersections, crosswalks, cameras, buses + the ~8 GB KYAPED LiDAR.
#
# The UE-derived georef base (MESHES/ + LIDAR/) is intentionally NOT baked into the
# image — only that local data can anchor the citywide build, so it is bind-mounted
# from the repo at runtime (see docker-compose.yml). The image carries just the code
# and Python/Chromium deps.
FROM python:3.12-slim

ENV PYTHONUNBUFFERED=1 \
    PIP_NO_CACHE_DIR=1 \
    PIP_DISABLE_PIP_VERSION_CHECK=1 \
    PLAYWRIGHT_BROWSERS_PATH=/ms-playwright

WORKDIR /app

# ca-certificates: HTTPS to OpenStreetMap / KyFromAbove S3 / Lextran during the build.
RUN apt-get update \
 && apt-get install -y --no-install-recommends ca-certificates \
 && rm -rf /var/lib/apt/lists/*

# Python deps first so this layer caches across source edits. requirements.txt includes
# the citywide extras (laspy[lazrs]/shapely/scikit-image) needed by build_all --citywide,
# plus playwright for the --render first-person cameras.
COPY requirements.txt ./
RUN pip install -r requirements.txt

# Headless Chromium for first-person agent cameras (--render). --with-deps apt-installs
# the browser's shared libraries.
RUN playwright install --with-deps chromium

# In-process YOLO detection (POST /api/cameras/detect): ultralytics + OpenCV. ultralytics
# pulls PyTorch — install the CPU wheel (this container has no GPU) to avoid ~2.5 GB of
# unused CUDA libraries, and opencv-headless so no GUI/GL system libs are needed on a server.
RUN pip install torch --index-url https://download.pytorch.org/whl/cpu \
 && pip install ultralytics opencv-python-headless

# App source. The big/generated paths (MESHES/, LIDAR/, web/data/, extracted/) are
# excluded by .dockerignore and supplied at runtime by the bind mount in compose.
COPY . .

EXPOSE 8000

# Default: bind all interfaces and auto-build a missing world without prompting. Bootstrap
# is idempotent — it short-circuits once manifest.json + ground.f32 + buildings.pack.json
# exist. Compose adds --render on top of this.
CMD ["python", "-m", "tools.twin_server", "--host", "0.0.0.0", "--bootstrap", "--render"]
