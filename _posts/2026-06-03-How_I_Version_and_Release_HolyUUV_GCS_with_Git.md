---
title: How I Version and Release HolyUUV_GCS with Git
date: 2026-06-03 04:00:00 +0900
categories: [Git, Workflow]
tags: [git, github, workflow, versioning, appimage, docker]
toc: true
comments: true
pin: true
math: false
mermaid: false
---
<hr>
<br>

> Personal memo so I don't forget how I built and shipped **HolyUUV_GCS**.
> `main` = release, `dev` = development. I develop on `dev` and merge into `main` only when shipping.

<br>
<hr>

## Dependency Installation

Everything is driven by the helper scripts in `scripts/`. Run them **in order** from the project root.

Step 1 installs all build dependencies (Qt 5 packages + MAVLink headers):

```bash
# Installs all Qt 5 / build deps via apt, then pulls the MAVLink C headers.
./scripts/install_deps.sh
```

| Script | What it does |
|--------|--------------|
| `scripts/install_deps.sh`        | Installs all Qt 5 / build dependencies via `apt`, then runs `setup_mavlink.sh` |
| `scripts/setup_mavlink.sh`       | Downloads the MAVLink C headers (`c_library_v2`) into `3rdparty/` |
| `scripts/build_project_linux.sh` | Clean CMake build → produces `build/HolyUUV_GCS` |
| `scripts/package_appimage.sh`    | *(optional)* Bundles a portable **AppImage** for distribution |

> The scripts can be run from anywhere — each one `cd`s to the project root automatically.

<br>
<hr>

## Clean Build

```bash
# 2. Clean CMake build (CMake + make).
./scripts/build_project_linux.sh

# 3. Run it.
./build/HolyUUV_GCS
```

> Officially developed and tested on **Ubuntu 24.04 (LTS)** only — Qt 5 / glibc versions differ on other releases.

<br>
<hr>

## Testing on Another Machine with Docker

When I'm not on Ubuntu 24.04, I spin up a clean container and build inside it to confirm the project still works on a fresh machine.

```bash
# On the host first, so the container can open windows:
xhost +local:docker

docker run -it --rm \
    --gpus all \                              # pass the GPU through (for 3D terrain)
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v "$(pwd)":/workspace \
    ubuntu:24.04 bash

# then, inside the container:
cd /workspace && ./scripts/install_deps.sh && cmake -B build && cmake --build build -j$(nproc)
```

> The AppImage is built on 24.04 (glibc 2.39), so it runs on **Ubuntu 24.04+**.
> Uses the system GPU when available, falls back to software rendering otherwise.

<br>
<hr>

## Git Branch Roles: `main` vs `dev`

| Branch | Role |
|--------|------|
| **`main`** | The release. Currently **v1.0.0**. Never commit here directly. |
| **`dev`**  | Where all development happens. Merge into `main` only when shipping. |

| Term | Meaning |
|------|---------|
| **Branch** (`main`, `dev`) | A development line that moves forward with every commit |
| **Tag** (`v1.0.0`) | A fixed name pinned to a specific commit — does not move |
| **merge** | Folding the changes from one branch into another |

<br>
<hr>

## What a Git Tag Means

A **tag** is a fixed label pinned to one specific commit, while a **branch** keeps moving forward. GitHub's **Releases** tab is built from tags — that's where I upload the AppImage.

```bash
# Tag the current release as v1.0.0 and push it
git tag v1.0.0
git push origin v1.0.0
```

### Version Number Rules (Semantic Versioning)

`vMAJOR.MINOR.PATCH` → e.g. `v1.2.3`

| Part | When to bump | Example |
|------|--------------|---------|
| MAJOR | Big change / breaks compatibility | 1.0.0 → 2.0.0 |
| MINOR | New feature (compatible) | 1.0.0 → 1.1.0 |
| PATCH | Bug fix only | 1.0.0 → 1.0.1 |

<br>
<hr>

## One-Time Setup (only the very first time)

```bash
cd ~/gcs_ws/HolyUUV_GCS

# Tag the current release as v1.0.0
git tag v1.0.0
git push origin v1.0.0

# Create the dev branch + push it to GitHub
git checkout -b dev
git push -u origin dev
```

<br>
<hr>

## Daily Development (on `dev` only)

```bash
git checkout dev          # make sure I'm on dev
# ... edit code ...
git add .
git commit -m "what changed"
git push                  # goes to dev (main untouched)
```

<br>
<hr>

## Shipping (merge `dev` → `main` + new version tag)

```bash
git checkout main
git merge dev             # bring dev changes into main
git tag v1.1.0            # new version (follow the number rules above)
git push origin main v1.1.0
git checkout dev          # back to dev to keep developing
```

<br>
<hr>

## AppImage Distribution (after a release)

```bash
# 1. Latest build + create the AppImage
cmake --build build -j$(nproc)
./scripts/package_appimage.sh        # → HolyUUV_GCS.AppImage

# 2. Attach the AppImage to that tag (v1.1.0) on GitHub Releases
#    GitHub web → Releases → Draft new release → pick the tag → upload the file
```

Running it needs no installation on **Ubuntu 24.04+**:

```bash
chmod +x HolyUUV_GCS.AppImage
./HolyUUV_GCS.AppImage
```

<br>
<hr>

## Frequently Used Git Commands (cheat sheet)

```bash
git status                # current state
git branch                # list branches (* = current)
git checkout dev          # switch to dev
git checkout main         # switch to main
git tag                   # list tags
git log --oneline -10     # last 10 commits
git checkout v1.0.0       # peek at the v1.0.0 release state (don't commit here)
```

<br>
<hr>

## Notes

- **Always develop on `dev`.** Never commit directly to `main`.
- Don't force-reset `main` to `dev` — rewriting history is dangerous. **Use merge.**
- Solo development can stay clean with `git merge dev --ff-only`.
