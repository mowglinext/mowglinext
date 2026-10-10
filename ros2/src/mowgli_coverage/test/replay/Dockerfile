# First build the repository's fields2cover-v3-builder target. This adds test tools
# without introducing another Fields2Cover source/version recipe.
ARG PLANNER_BASE=mowgli-f2c-replay:884d895
FROM ${PLANNER_BASE}
USER root
RUN apt-get update && apt-get install -y --no-install-recommends \
    libgtest-dev clang-format-18 time \
 && rm -rf /var/lib/apt/lists/*
USER ubuntu
WORKDIR /work
