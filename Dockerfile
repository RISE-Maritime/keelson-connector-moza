# evdev ships only as an sdist and builds against the kernel's input headers: build the
# wheel here, so the runtime image carries no compiler.
FROM python:3.13-slim-bookworm AS build
RUN apt-get update \
    && apt-get install -y --no-install-recommends gcc libc6-dev linux-libc-dev \
    && rm -rf /var/lib/apt/lists/*
COPY --from=ghcr.io/astral-sh/uv:latest /uv /bin/
WORKDIR /build
COPY pyproject.toml uv.lock README.md ./
COPY moza_connector moza_connector
# The exact versions uv.lock pins -- keelson from PyPI at its exact pin, never a local checkout.
RUN uv export --frozen --no-emit-project --no-hashes --format requirements-txt -o requirements.txt \
    && pip wheel --no-cache-dir --no-deps --wheel-dir /wheels -r requirements.txt . 

FROM python:3.13-slim-bookworm
RUN apt-get update \
    && apt-get install -y --no-install-recommends tini \
    && rm -rf /var/lib/apt/lists/*
COPY --from=build /wheels /wheels
RUN pip install --no-cache-dir --no-index --find-links /wheels /wheels/*.whl && rm -rf /wheels

# Reading /dev/input/event* needs the device's group (input) -- docker-compose.moza.yml
# adds it with group_add; the user itself is unprivileged.
RUN useradd --uid 10001 --no-create-home --shell /usr/sbin/nologin app
USER app

ENTRYPOINT ["/usr/bin/tini", "-g", "--", "/bin/bash", "-c"]
