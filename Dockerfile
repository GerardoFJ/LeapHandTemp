# Initial Arguments
ARG WINE_VERSION=11.16~noble-1
ARG WINE_TAG=wine-11.16
ARG WINE_REPO=GerardoFJ/wine
ARG WINE_BRANCH=haptx-usb

# ---------------------------------------------------------------------------
# Build the patched USB modules from the Wine fork (WINE_REPO@WINE_BRANCH).
# ---------------------------------------------------------------------------
FROM ubuntu:24.04 AS wineusb

ARG WINE_TAG
ARG WINE_REPO
ARG WINE_BRANCH
ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
        git ca-certificates build-essential flex bison pkg-config \
        gcc-mingw-w64-x86-64 g++-mingw-w64-x86-64 \
        libusb-1.0-0-dev libfreetype-dev libx11-dev libxext-dev \
    && rm -rf /var/lib/apt/lists/*

### WINE SOURCE
# The Wine fork's branch carries the USB changes on top of ${WINE_TAG}.
# Fetching the branch head first busts the cache when the branch moves, so a
# new push is picked up without --no-cache.
ADD https://api.github.com/repos/${WINE_REPO}/git/refs/heads/${WINE_BRANCH} /tmp/wine-ref.json
RUN git clone --depth 1 --branch "${WINE_BRANCH}" \
        "https://github.com/${WINE_REPO}.git" /opt/wine-src \
    && echo "wine fork: $(git -C /opt/wine-src log --oneline -1)" \
    && got="$(cat /opt/wine-src/VERSION)" \
    && case "$got" in \
         "Wine version ${WINE_TAG#wine-}") echo "wine source: $got" ;; \
         *) echo "ERROR: ${WINE_REPO}@${WINE_BRANCH} is '$got' but WINE_TAG is ${WINE_TAG}" >&2; exit 1 ;; \
       esac

## COMPILE WINE
RUN cd /opt/wine-src \
    && ./configure --enable-win64 \
        CPPFLAGS='-I/usr/include/libusb-1.0 -I/usr/include/freetype2'

RUN cd /opt/wine-src && make -j"$(nproc)" \
        dlls/wineusb.sys/x86_64-windows/wineusb.sys \
        dlls/wineusb.sys/wineusb.so \
        dlls/winusb/x86_64-windows/winusb.dll \
        dlls/setupapi/x86_64-windows/setupapi.dll \
        dlls/cfgmgr32/x86_64-windows/cfgmgr32.dll

RUN mkdir -p /out/pe /out/unix \
    && cd /opt/wine-src \
    && cp dlls/wineusb.sys/x86_64-windows/wineusb.sys \
          dlls/winusb/x86_64-windows/winusb.dll \
          dlls/setupapi/x86_64-windows/setupapi.dll \
          dlls/cfgmgr32/x86_64-windows/cfgmgr32.dll /out/pe/ \
    && cp dlls/wineusb.sys/wineusb.so /out/unix/

# ---------------------------------------------------------------------------
# Fetch the MSVC toolchain.
# MSVC, run under Wine by msvc-wine.
# ---------------------------------------------------------------------------
FROM ubuntu:24.04 AS msvc

ARG DEBIAN_FRONTEND=noninteractive
ARG MSVC_PACKAGES="--architecture x64 --with-default no --with-msvc yes --with-sdk yes --with-atl no --with-asan no"

RUN apt-get update && apt-get install -y --no-install-recommends \
        git ca-certificates python3 msitools \
    && rm -rf /var/lib/apt/lists/*

RUN git clone --depth 1 https://github.com/mstorsjo/msvc-wine /opt/msvc-wine

# --accept-license accepts Microsoft's Visual Studio Build Tools licence.
RUN python3 /opt/msvc-wine/vsdownload.py --accept-license ${MSVC_PACKAGES} --dest /opt/msvc \
 && test -d /opt/msvc/VC

# ---------------------------------------------------------------------------
# Install wine stable .
# ---------------------------------------------------------------------------
FROM ubuntu:24.04

ARG WINE_VERSION
ARG WINE_TAG
ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
        wget ca-certificates gnupg \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /etc/apt/keyrings \
    && wget -O - https://dl.winehq.org/wine-builds/winehq.key \
       | gpg --dearmor -o /etc/apt/keyrings/winehq-archive.key \
    && dpkg --add-architecture i386 \
    && wget -NP /etc/apt/sources.list.d/ \
       https://dl.winehq.org/wine-builds/ubuntu/dists/noble/winehq-noble.sources

RUN apt-get update && apt-get install -y --install-recommends \
        "winehq-staging=${WINE_VERSION}" \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends \
        libusb-1.0-0 usbutils procps python3 binutils \
        x11-utils x11-xserver-utils xdotool xauth \
    && rm -rf /var/lib/apt/lists/*

## PATCH THE WINE FILES

COPY --from=wineusb /out/pe/   /opt/wine-staging/lib/wine/x86_64-windows/
COPY --from=wineusb /out/unix/ /opt/wine-staging/lib/wine/x86_64-unix/

ENV PATH="/opt/wine-staging/bin:${PATH}" \
    WINEPREFIX=/root/.wine \
    WINEARCH=win64 \
    WINEDLLOVERRIDES="mscoree,mshtml=" \
    WINEDEBUG=-all

# Fail the build loudly if the source and the package are not the same release.
RUN set -eu; \
    got="$(wine --version)"; \
    want="wine-${WINE_TAG#wine-}"; \
    case "$got" in \
      "$want"*) echo "wine $got matches ${WINE_TAG}" ;; \
      *) echo "ERROR: built ${WINE_TAG} but package is '$got'" >&2; exit 1 ;; \
    esac

COPY scripts/killall.sh              /usr/local/bin/haptx-killall.sh
COPY scripts/start-dashboard.sh      /usr/local/bin/haptx-dashboard.sh
COPY scripts/bind-devices.sh         /usr/local/bin/haptx-bind.sh
COPY scripts/entrypoint.sh           /usr/local/bin/haptx-entrypoint.sh
COPY scripts/bind-haptx-devices.py   /usr/local/share/haptx/bind-haptx-devices.py
COPY scripts/build-cpp.sh            /usr/local/bin/haptx-build.sh
RUN chmod +x /usr/local/bin/haptx-killall.sh \
             /usr/local/bin/haptx-dashboard.sh \
             /usr/local/bin/haptx-bind.sh \
             /usr/local/bin/haptx-entrypoint.sh \
             /usr/local/bin/haptx-build.sh

# ---------------------------------------------------------------------------
# Wire up MSVC.
# ---------------------------------------------------------------------------
COPY --from=msvc /opt/msvc-wine /opt/msvc-wine
COPY --from=msvc /opt/msvc      /opt/msvc

#
RUN WINEPREFIX=/root/.wine-msvc WINEDEBUG=-all /opt/msvc-wine/install.sh /opt/msvc \
 && test -x /opt/msvc/bin/x64/cl \
 && test -x /opt/msvc/bin/msvctricks.exe \
 && rm -rf /root/.wine-msvc \
 && echo "msvc ready: $(du -sh /opt/msvc | cut -f1)"

RUN --mount=type=bind,source=sdk/haptx-prefix.tar.gz,target=/tmp/prefix.tar.gz \
    tar -C /root -xzf /tmp/prefix.tar.gz \
 && test -f "/root/.wine/drive_c/Program Files/HaptX/SDK/Tools/HaptxDashboard/HaptxDashboard.exe" \
 && test -f "/root/.wine/drive_c/ProgramData/HaptX/SDK/username.txt" \
 && echo "prefix unpacked, $(du -sh /root/.wine | cut -f1), user $(cat '/root/.wine/drive_c/ProgramData/HaptX/SDK/username.txt')"

ENTRYPOINT ["/usr/local/bin/haptx-entrypoint.sh"]
CMD ["sleep", "infinity"]
