FROM golang:1.24 AS build-go

RUN useradd --create-home --uid 1000 builder
COPY --chown=builder:builder . /app
WORKDIR /app
USER builder
ARG BUILD_REVISION
ARG BUILD_VERSION
ARG BUILD_TIME
RUN CGO_ENABLED=0 go build -ldflags "-X github.com/mowglinext/mowglinext/pkg/buildinfo.Revision=${BUILD_REVISION} -X github.com/mowglinext/mowglinext/pkg/buildinfo.Version=${BUILD_VERSION} -X github.com/mowglinext/mowglinext/pkg/buildinfo.BuiltAt=${BUILD_TIME}" -o mowglinext

FROM node:22 AS build-web
COPY --chown=node:node ./web /web
WORKDIR /web
USER node
ARG BUILD_REVISION
ARG BUILD_VERSION
ARG BUILD_TIME
RUN yarn && VITE_BUILD_REVISION="$BUILD_REVISION" VITE_BUILD_VERSION="$BUILD_VERSION" VITE_BUILD_TIME="$BUILD_TIME" yarn build \
 && find dist/assets -type f \( -name '*.js' -o -name '*.css' -o -name '*.json' -o -name '*.svg' -o -name '*.wasm' \) \
      -size +1024c -exec gzip -9 -k {} +

FROM ubuntu:22.04 AS deps
RUN apt-get update && apt-get install -y ca-certificates curl python3 python3-pip python3-venv libjim-dev util-linux\
                                      git build-essential unzip wget autoconf automake pkg-config texinfo libtool libftdi-dev libusb-1.0-0-dev
RUN apt-get install -y rpi.gpio-common  || true
# openocd recursively pulls its libjaylink submodule from gitlab.zapb.de, which
# is frequently unreachable (504 / timeouts) and hard-fails the image build.
# Redirect that submodule to a GitHub mirror (verified to carry the pinned
# commit) and retry the clone a few times for transient blips.
# PINNED, and it must stay pinned. `--branch rpi-common --depth=1` tracked a
# MOVING branch, so the OpenOCD in the image was whatever that branch happened
# to be on build day. Two robots installed weeks apart got materially different
# flashing behaviour with no commit in this repo to explain it — which is
# exactly how users hit "Debug adapter doesn't support 'swd' transport" while
# the maintainer could not reproduce it.
#
# This binary is what BOTH flash paths in gui/pkg/providers/firmware.go run
# (bare `openocd`, resolved on PATH), so its ST-Link support decides whether a
# user can flash at all.
ARG OPENOCD_REF=acff23ffd100479d6481a1155123774b79a990b9
RUN git config --global url."https://github.com/syntacore/libjaylink.git".insteadOf "https://gitlab.zapb.de/libjaylink/libjaylink.git" \
 && for i in 1 2 3; do \
      rm -rf openocd && \
      git clone --branch rpi-common https://github.com/raspberrypi/openocd.git && \
      git -C openocd checkout --detach "$OPENOCD_REF" && \
      git -C openocd submodule update --init --recursive && break || \
      { echo "openocd clone attempt $i failed, retrying..."; sleep 10; }; \
    done
RUN cd openocd && ./bootstrap with-submodules && ./configure --enable-ftdi --enable-sysfsgpio --enable-bcm2835gpio && make -j$(nproc) && make install && cd .. && rm -rf openocd
# Fail the BUILD, not the user's robot. An OPENOCD_REF bump that loses ST-Link
# support, or ships a driver with no SWD transport at all, would break flashing
# for every user with no other warning. Accepts `swd` (dapdirect) OR `hla_swd`
# on purpose: we deliberately let openocd negotiate the transport rather than
# forcing one, because dapdirect SWD needs recent dongle firmware on an ST-Link
# V2 while a V3 always has it. Requiring `swd` here would re-impose exactly the
# V2 incompatibility this setup exists to avoid.
#
# `transport list` only prints the driver's capabilities and never initialises
# the adapter, so this is safe with no hardware attached.
RUN transports="$(openocd -f interface/stlink.cfg -c 'transport list' -c shutdown 2>&1)"; \
    echo "$transports"; \
    echo "$transports" | grep -qE '^[[:space:]]*(hla_)?swd$' || { \
      echo "FATAL: this openocd's ST-Link driver offers no SWD transport."; \
      echo "Neither 'swd' nor 'hla_swd' was advertised, so no ST-Link V2 or V3"; \
      echo "could flash the board. Pick a different OPENOCD_REF."; \
      exit 1; }
RUN curl -fsSL https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py -o get-platformio.py &&    python3 get-platformio.py
RUN mkdir -p /usr/local/bin &&    ln -s ~/.platformio/penv/bin/platformio /usr/local/bin/platformio &&    ln -s ~/.platformio/penv/bin/pio /usr/local/bin/pio &&    ln -s ~/.platformio/penv/bin/piodebuggdb /usr/local/bin/piodebuggdb

# Pre-fetch PlatformIO platform, toolchain and framework so firmware
# flashing doesn't download ~200 MB on first use.
# Use the exact toolchain version pinned by the Mowgli firmware.
# The --skip-dependencies flag avoids pulling the default toolchain
# (which may not exist for all architectures).
RUN pio pkg install -g \
      -p "platformio/ststm32" \
      -t "platformio/tool-stm32duino" \
      -t "platformio/tool-openocd" \
      --skip-dependencies \
 && pio pkg install -g \
      -t "toolchain-gccarmnoneeabi@~1.90301.0" \
      --skip-dependencies \
    || true

FROM deps
COPY --from=build-web /web/dist /app/web
COPY --from=build-go /app/mowglinext /app/mowglinext
COPY asserts /app/asserts
ENV WEB_DIR=/app/web
ENV DB_PATH=/app/db
WORKDIR /app
CMD ["/app/mowglinext"]
