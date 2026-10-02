# HaptX G1 on Linux, under Wine

Self-contained Docker setup that runs the Windows-only HaptX SDK 3.0.3 under
Wine and reaches the G1 gloves and the Airpack over **USB**.

```
haptx-docker/
├── Dockerfile              two-stage: build patched Wine modules, then runtime
├── docker-compose.yml      the service, with every mount that matters
├── sdk/                    the exported prefix archive (gitignored, ~800 MB)
└── scripts/
    ├── entrypoint.sh            runs at container start; calls bind-devices.sh
    ├── bind-devices.sh          bind attached devices (also runnable by hand)
    ├── bind-haptx-devices.py    the binder itself, port-independent
    ├── start-dashboard.sh       start the Dashboard correctly
    ├── build-cpp.sh             build a C++ client against the SDK
    ├── killall.sh               stop Wine safely
    ├── check.sh                 host: smoke-test a running container
    └── export-prefix.sh         re-export the prefix for a newer SDK
```

## Setup

**1. Nothing, usually.** The image ships the SDK, so there is no installer to
point at and no X cookie to generate.

Optionally, if you want to reach host files from inside the container -- client
apps such as `CppStarterProject`, or the SDK installer when re-exporting a
prefix for a newer SDK -- set `HAPTX_WORKDIR`:

```bash
cd haptx-docker
echo "HAPTX_WORKDIR=/home/jrlgerardo/TempDockerfiles" > .env
```

**2. Build and start** (the build takes ~20 minutes; it compiles Wine modules):

```bash
docker compose build
docker compose up -d
scripts/check.sh
```

## Running

```bash
docker compose exec haptx haptx-dashboard.sh
```

## Binding, and when to re-run it

The entrypoint binds every attached HaptX device at container start
Re-run it by hand if you move a device to a different port while the container
is running:

```bash
docker compose exec haptx haptx-bind.sh
```


## Building C++ clients

The image ships MSVC, so a client app builds inside the container:

```bash
docker compose exec haptx haptx-build.sh /workdir/myapp.cpp
docker compose exec -w /workdir haptx wine myapp.exe
```

It writes `myapp.exe` next to the source and copies the two DLLs it needs
beside it.

**GCC and MinGW cannot build against this SDK**, which is why MSVC is in the
image at all. `HaptxApi.dll` exports MSVC-mangled C++ names:

