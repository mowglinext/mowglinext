FROM golang:1.24 AS build-go

COPY . /app
WORKDIR /app
RUN CGO_ENABLED=0 go build -o mowglinext

FROM node:22 AS build-web
COPY ./web /web
WORKDIR /web
RUN yarn && yarn build

FROM ubuntu:22.04 AS deps
RUN apt-get update && apt-get install -y ca-certificates curl python3 python3-pip python3-venv libjim-dev\
                                      git build-essential unzip wget autoconf automake pkg-config texinfo libtool libftdi-dev libusb-1.0-0-dev
RUN apt-get install -y rpi.gpio-common  || true
# openocd recursively pulls its libjaylink submodule from gitlab.zapb.de, which
# is frequently unreachable (504 / timeouts) and hard-fails the image build.
# Redirect that submodule to a GitHub mirror (verified to carry the pinned
# commit) and retry the clone a few times for transient blips.
RUN git config --global url."https://github.com/syntacore/libjaylink.git".insteadOf "https://gitlab.zapb.de/libjaylink/libjaylink.git" \
 && for i in 1 2 3; do \
      rm -rf openocd && \
      git clone --recursive --branch rpi-common --depth=1 https://github.com/raspberrypi/openocd.git && break || \
      { echo "openocd clone attempt $i failed, retrying..."; sleep 10; }; \
    done
RUN cd openocd && ./bootstrap with-submodules && ./configure --enable-ftdi --enable-sysfsgpio --enable-bcm2835gpio && make -j$(nproc) && make install && cd .. && rm -rf openocd
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
