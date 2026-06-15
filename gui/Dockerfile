FROM golang:1.24 AS build-go

COPY . /app
WORKDIR /app
RUN CGO_ENABLED=0 go build -o mowglinext

FROM node:22 AS build-web
COPY ./web /web
WORKDIR /web
RUN yarn && yarn build

FROM ubuntu:22.04 AS deps
RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates curl python3 python3-pip python3-venv \
    git build-essential unzip wget file \
    stlink-tools libusb-1.0-0 \
    gcc-arm-none-eabi binutils-arm-none-eabi \
 && rm -rf /var/lib/apt/lists/*
RUN curl -fsSL https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py -o get-platformio.py &&    python3 get-platformio.py
RUN mkdir -p /usr/local/bin \
 && ln -s /root/.platformio/penv/bin/platformio /usr/local/bin/platformio \
 && ln -s /root/.platformio/penv/bin/pio /usr/local/bin/pio \
 && ln -s /root/.platformio/penv/bin/piodebuggdb /usr/local/bin/piodebuggdb
 
# Pre-fetch PlatformIO platform, toolchain and framework so firmware
# flashing doesn't download ~200 MB on first use.
# Use the exact toolchain version pinned by the Mowgli firmware.
# The --skip-dependencies flag avoids pulling the default toolchain
# (which may not exist for all architectures).
RUN pio pkg install -g \
      -p "platformio/ststm32" \
      -t "platformio/tool-stm32duino" \
      -t "platformio/tool-dfuutil" \
      -t "platformio/tool-stm32flash" \
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
