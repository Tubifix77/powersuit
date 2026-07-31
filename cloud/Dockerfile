# syntax=docker/dockerfile:1
#
# Build context MUST be the repository root, not cloud/, so that
# ../common/python (the shared powersuit_proto protocol library cloud_ai_core
# depends on — docs/link-protocol.md, docs/network-map.md) is reachable from
# this Dockerfile. Build/run from the repo root, e.g.:
#   docker build --target test -f cloud/Dockerfile .
#   docker build -t cloud-ai-core -f cloud/Dockerfile .
#   docker run --rm -p 8765:8765 -e CLOUD_LINK_TOKENS=dev-token cloud-ai-core
# (docker-compose.yml already sets the build context/dockerfile for you.)

FROM python:3.12-slim AS deps
WORKDIR /app

# Shared protocol library first (rarely changes -> better layer caching),
# then the cloud package itself.
COPY common/python ./common/python
COPY cloud/pyproject.toml ./cloud/pyproject.toml
COPY cloud/cloud_ai_core ./cloud/cloud_ai_core

# Editable installs: cloud_ai_core.config resolves its default RAG docs dir
# relative to its OWN source tree (parent.parent / "docs" / "manuals"), so it
# must keep pointing at /app/cloud rather than a site-packages copy.
RUN pip install --no-cache-dir -e ./common/python -e ./cloud

# ---------------------------------------------------------------------------
FROM deps AS test

# Match cloud/pyproject.toml's [project.optional-dependencies].dev without
# disturbing the editable install done above.
RUN pip install --no-cache-dir "pytest>=8,<9" "pytest-asyncio>=1.4,<2"

COPY cloud/docs ./cloud/docs
COPY cloud/tests ./cloud/tests

RUN python -m pytest cloud/tests -q

# ---------------------------------------------------------------------------
FROM deps AS final

COPY cloud/docs ./cloud/docs

ENV CLOUD_BIND=0.0.0.0:8765
EXPOSE 8765

ENTRYPOINT ["cloud-ai-core"]
