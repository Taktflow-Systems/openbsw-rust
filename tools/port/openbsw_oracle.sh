#!/usr/bin/env bash
# Linux-native equivalent of openbsw_oracle.ps1 for hosts without WSL.
# Builds the pinned upstream oracle checkout in place; the checkout lives on
# a native filesystem already, so no disposable mirror is needed.
#
# Post-drift upstream (2026-06-02 tip): minimum C++17, Bazel-primary layout
# with CMake presets retained, and middleware build-time code generation
# (jinja2cpp.py via add_custom_command; needs python3 with jinja2 + yaml).
# The CMake preset path remains the supported oracle build; Bazel is not
# required to build or test the oracle.
set -euo pipefail

ACTION="${1:-Test}"
JOBS="${2:-4}"
PINNED_COMMIT='be0029bbb79fe901048a24c2665f2ba854328734'
REPOSITORY='https://github.com/eclipse-openbsw/openbsw.git'
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
CHECKOUT="$REPO_ROOT/target/oracle/openbsw"

case "$ACTION" in
    Checkout|Configure|Build|Test|Reference|All) ;;
    *) echo "usage: $0 [Checkout|Configure|Build|Test|Reference|All] [jobs]" >&2; exit 2 ;;
esac

if [ ! -d "$CHECKOUT/.git" ]; then
    mkdir -p "$(dirname "$CHECKOUT")"
    # Prefer the local read-only drift mirror when present; fall back to the
    # public repository. Both must still resolve the pinned SHA below.
    if [ -d "$REPO_ROOT/target/drift/openbsw-drift.git" ]; then
        git clone --no-checkout "$REPO_ROOT/target/drift/openbsw-drift.git" "$CHECKOUT"
    else
        git clone --filter=blob:none --no-checkout "$REPOSITORY" "$CHECKOUT"
    fi
fi

CURRENT="$(git -C "$CHECKOUT" rev-parse HEAD 2>/dev/null || true)"
if [ "$CURRENT" != "$PINNED_COMMIT" ]; then
    git -C "$CHECKOUT" fetch origin "$PINNED_COMMIT" || true
    git -C "$CHECKOUT" checkout --detach "$PINNED_COMMIT"
fi

VERIFIED="$(git -C "$CHECKOUT" rev-parse HEAD)"
if [ "$VERIFIED" != "$PINNED_COMMIT" ]; then
    echo "Oracle HEAD $VERIFIED is not pinned commit $PINNED_COMMIT" >&2
    exit 1
fi
if [ "$ACTION" = "Checkout" ]; then
    echo "Pinned OpenBSW checkout ready: $CHECKOUT"
    exit 0
fi

python3 -c 'import jinja2, yaml' 2>/dev/null || {
    echo 'python3 with jinja2 and yaml is required for middleware build-time code generation' >&2
    exit 1
}

cd "$CHECKOUT"
case "$ACTION" in Configure|Build|Test|All)
    cmake --preset tests-posix-debug ;;
esac
case "$ACTION" in Build|Test|All)
    cmake --build --preset tests-posix-debug --parallel "$JOBS" ;;
esac
case "$ACTION" in Test|All)
    ctest --preset tests-posix-debug --output-on-failure ;;
esac
case "$ACTION" in Reference|All)
    cmake --preset posix-freertos
    cmake --build --preset posix-freertos --parallel "$JOBS"
    timeout --signal=TERM 5s \
        build/posix-freertos/executables/referenceApp/application/Release/app.referenceApp.elf \
        </dev/null || test $? -eq 124 ;;
esac
