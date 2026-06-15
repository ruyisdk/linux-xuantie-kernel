#!/usr/bin/env bash
# SPDX-License-Identifier: MIT
#
# CI orchestrator for the C930 + linux-64lp64-test test suite.
#
# What it does:
#   1.  Boot QEMU with virtio-9p host-share exposing the kernel-build artifacts
#       (modules_install/ and kernel_testsuite/ runner scripts).
#   2.  Drive the in-guest setup via .github/ci_scripts/check_qemu.exp:
#         - bring up the network
#         - apt-get update
#   3.  Replace /lib/modules/<KVER>/ with the freshly compiled tree via 9p
#       (.github/ci_scripts/inject_modules.exp), guaranteeing modules match Image.
#   4.  Provision /etc/init.ci/kernel_testsuite/ runner scripts from 9p host-share.
#   5.  For each test case, apt-install the test deps and execute the runner.
#
# Usage:
#   run_xt_test_suite.sh \
#       --modules-dir ./modules_install [--filter vcrypto-all,kselftest-riscv]
#
# Exit code: 0 if every selected case passed, non-zero if any failed.

set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CHECK_QEMU="${SCRIPT_DIR}/check_qemu.exp"
INJECT_MOD="${SCRIPT_DIR}/inject_modules.exp"

# ---- defaults --------------------------------------------------------------
CPU="${CPU:-xt-c930v-cp}"
SMP="${SMP:-4}"
MEM="${MEM:-4G}"
IMAGE="${IMAGE:-./Image}"
ROOTFS="${ROOTFS:-./rootfs_rv64.ext4}"
BIOS="${BIOS:-./fw_dynamic.bin}"
MODULES_DIR="${MODULES_DIR:-./modules_install}"
TELNET_PORT="${TELNET_PORT:-5678}"
SSH_PORT="${SSH_PORT:-2222}"
SKIP_BOOT="${SKIP_BOOT:-0}"
TEST_FILTER="${TEST_FILTER:-}"
QEMU_BIN="${QEMU_BIN:-qemu-system-riscv64}"

# ---- arg parsing -----------------------------------------------------------
while [[ $# -gt 0 ]]; do
    case "$1" in
        --cpu)             CPU="$2"; shift 2 ;;
        --smp)             SMP="$2"; shift 2 ;;
        --mem)             MEM="$2"; shift 2 ;;
        --image)           IMAGE="$2"; shift 2 ;;
        --rootfs)          ROOTFS="$2"; shift 2 ;;
        --bios)            BIOS="$2"; shift 2 ;;
        --modules-dir)     MODULES_DIR="$2"; shift 2 ;;
        --qemu)            QEMU_BIN="$2"; shift 2 ;;
        --skip-boot)       SKIP_BOOT=1; shift ;;
        --filter)          TEST_FILTER="$2"; shift 2 ;;
        -h|--help)
            sed -n '2,22p' "$0"; exit 0 ;;
        *) echo "Unknown arg: $1" >&2; exit 1 ;;
    esac
done

log()  { printf '[run_xt_test_suite] %s\n' "$*"; }
fail() { printf '[run_xt_test_suite][FAIL] %s\n' "$*" >&2; exit 1; }

[[ -x "$CHECK_QEMU"   ]] || fail "missing $CHECK_QEMU"
[[ -x "$INJECT_MOD"   ]] || fail "missing $INJECT_MOD"

HOST_SHARE_DIR="$(cd "$(dirname "$MODULES_DIR")" && pwd)/$(basename "$MODULES_DIR")"
[[ -d "$HOST_SHARE_DIR" ]] || fail "modules_install dir not found: $MODULES_DIR"
# QEMU 9p shares a directory; the inject helper looks for
# $GMOUNT/modules_install/lib/...  Share the parent of modules_install.
HOST_SHARE_PARENT="$(dirname "$HOST_SHARE_DIR")"
log "9p host-share root  : $HOST_SHARE_PARENT (exposes modules_install/...)"

# Ensure kernel_testsuite runners are available in the 9p host-share
RUNNERS_SRC="${SCRIPT_DIR}/kernel_testsuite"
RUNNERS_DST="${HOST_SHARE_PARENT}/kernel_testsuite"
if [[ -d "$RUNNERS_SRC" ]]; then
    if [[ ! -d "$RUNNERS_DST" ]] || [[ "$RUNNERS_SRC" != "$RUNNERS_DST" ]]; then
        cp -a "$RUNNERS_SRC" "$RUNNERS_DST" 2>/dev/null
    fi
    log "kernel_testsuite runners staged at: $RUNNERS_DST"
else
    log "WARN: no kernel_testsuite/ found at $RUNNERS_SRC"
fi

# ---- launch QEMU -----------------------------------------------------------
QEMU_LOG="${QEMU_LOG:-./qemu-${CPU}.log}"
if [[ "$SKIP_BOOT" != "1" ]]; then
    log "launching QEMU ($QEMU_BIN -cpu $CPU -smp $SMP -m $MEM)"
    "$QEMU_BIN" -nographic -cpu "$CPU" -smp "$SMP" -m "$MEM" -M virt \
        -bios "$BIOS" -kernel "$IMAGE" \
        -append 'rootwait root=/dev/vda rw' \
        -drive "file=${ROOTFS},format=raw,id=hd0" -device virtio-blk-device,drive=hd0 \
        -netdev "user,id=net0,hostfwd=tcp::${SSH_PORT}-:22" \
        -device virtio-net-device,netdev=net0 \
        -fsdev "local,security_model=mapped-xattr,id=fsdev0,path=${HOST_SHARE_PARENT}" \
        -device virtio-9p-pci,id=fs0,fsdev=fsdev0,mount_tag=hostshare \
        -monitor none -serial "telnet:localhost:${TELNET_PORT},server,nowait" \
        > "$QEMU_LOG" 2>&1 &
    QEMU_PID=$!
    log "QEMU pid=$QEMU_PID, log=$QEMU_LOG"
    trap '[[ -n "${QEMU_PID:-}" ]] && kill "$QEMU_PID" 2>/dev/null || true' EXIT

    # Wait for QEMU telnet serial port to become available
    log "waiting for QEMU telnet port ${TELNET_PORT}..."
    for i in $(seq 1 60); do
        if bash -c "echo >/dev/tcp/localhost/${TELNET_PORT}" 2>/dev/null; then
            log "telnet port ready after ${i}s"
            break
        fi
        sleep 1
    done

    log "waiting for kernel boot..."
    "$CHECK_QEMU" 'cat /proc/version' 240 || fail "QEMU did not boot"
fi

# ---- in-guest setup --------------------------------------------------------
log "bring up network"
NET_CMD='modprobe virtio_net 2>/dev/null;'
NET_CMD+=' ip link set eth0 up && udhcpc -i eth0 -t 5 -n || dhclient eth0;'
NET_CMD+=' ip a'
"$CHECK_QEMU" "$NET_CMD" 90

log "apt-get update"
"$CHECK_QEMU" 'apt-get update' 600 || log "WARN: apt-get update reported errors"

log "inject freshly compiled kernel modules"
"$INJECT_MOD" hostshare /mnt/hostshare \
    || fail "inject_modules failed (kernel-module replacement step)"

# Provision runner scripts: copy from 9p host-share into guest
log "provision test runner scripts from 9p host-share"
PROV_CMD='mkdir -p /etc/init.ci/kernel_testsuite'
PROV_CMD+=' && cp -a /mnt/hostshare/kernel_testsuite/* /etc/init.ci/kernel_testsuite/'
PROV_CMD+=' && chmod +x /etc/init.ci/kernel_testsuite/*'
PROV_CMD+=' && ls /etc/init.ci/kernel_testsuite/ | wc -l'
"$CHECK_QEMU" "$PROV_CMD" 120 \
    || fail "failed to provision runner scripts from 9p"

# ---- test cases ------------------------------------------------------------
# Format per row (tab-separated):
#   id  apt_pkgs  run_cmd  timeout_sec  pass_re
# Note: run_cmd is relative to RUNNER_DIR; fail_re defaults to "ERROR!"
RUNNER_DIR=/etc/init.ci/kernel_testsuite
DFLT_FAILRE="ERROR!"
read -r -d '' CASE_TABLE <<'CASEEOF' || true
kselftest-cgroup	perl* kernel-selftest util-linux	kselftest_run CGROUP	7200	KSELFTEST TEST cgroup PASSED
kselftest-fs	perl* kernel-selftest util-linux-taskset	kselftest_run FS	7200	KSELFTEST TEST filesystems PASSED
kselftest-ftrace	perl* kernel-selftest util-linux-taskset	kselftest_run FTRACE	7200	KSELFTEST TEST ftrace PASSED
kselftest-hotplug	perl* kernel-selftest util-linux-taskset	kselftest_run HOTPLUG	7200	KSELFTEST TEST hotplug PASSED
kselftest-mm	perl* kernel-selftest util-linux-taskset	kselftest_run MM	7200	KSELFTEST TEST mm PASSED
kselftest-pid	perl* kernel-selftest util-linux-taskset	kselftest_run PID	7200	KSELFTEST TEST pid_namespace PASSED
kselftest-riscv	perl* kernel-selftest util-linux-taskset	kselftest_run RISCV	7200	KSELFTEST TEST riscv PASSED
kselftest-syscall	perl* kernel-selftest util-linux-taskset	kselftest_run SYSCALL	7200	KSELFTEST TEST syscall_user_dispatch PASSED
ltp-container	inetutils-telnet ltp numa* kernel-module-nls* kernel-module-veth*	ltp_run CONTAINER	14400	LTP TEST container PASSED
ltp-misc	inetutils-telnet ltp numa* kernel-module-nls* kernel-module-veth* kernel-module-loop*	ltp_run MISC	7200	LTP TEST misc PASSED
bitops	bc kernel-module-test-bitops* kernel-module-test-string*	bitops_run	1800	bitops TEST PASSED
random	bc	random_run	1800	ZKR TEST PASSED
perf-record	perf	perf_record_run	1800	PERF RECORD TEST PASSED
perf-extension-all	_	perf-extension-check_run ALL	1800	ALL EXTENSIONS TEST PASSED
vcrypto-all	kernel-module-aes* kernel-module-sm* kernel-module-sha* kernel-module-ghash* kernel-module-chacha* kernel-module-tcrypt* kernel-module-cbc* kernel-module-ctr*	vcrypto_run ALL	1800	vcrypto TEST all PASSED
CASEEOF

declare -a PASSED=() FAILED=() SKIPPED=()
declare -a FILTER_LIST=()
if [[ -n "$TEST_FILTER" ]]; then
    IFS=',' read -r -a FILTER_LIST <<< "$TEST_FILTER"
fi

want_run() {
    local id="$1"
    [[ ${#FILTER_LIST[@]} -eq 0 ]] && return 0
    for f in "${FILTER_LIST[@]}"; do [[ "$f" == "$id" ]] && return 0; done
    return 1
}

# ---- batch apt install -----------------------------------------------------
# Collect every apt pkg from the cases that will run, dedupe, and install in
# a single guest-side apt-get call.  This avoids paying a 1st-time download +
# unpack per case (the cgroup case alone has hit our 900s per-case timeout).
# Per-case apt install is still kept below as an idempotent safety net.
ALL_PKGS=""
while IFS=$'\t' read -r ID PKGS _CMD _TMOUT _PASS _FAIL; do
    [[ -z "${ID:-}" ]] && continue
    if ! want_run "$ID"; then continue; fi
    [[ -z "${PKGS// }" || "$PKGS" == "_" ]] && continue
    ALL_PKGS+=" $PKGS"
done <<< "$CASE_TABLE"
# Dedup whitespace-separated pkg list.
ALL_PKGS="$(printf '%s\n' $ALL_PKGS | awk 'NF && !seen[$0]++' | tr '\n' ' ')"
HAS_KMOD=0
if [[ "$ALL_PKGS" == *kernel-module-* ]]; then HAS_KMOD=1; fi
HAS_KSELF=0
if [[ "$ALL_PKGS" == *kernel-selftest* ]]; then HAS_KSELF=1; fi

if [[ -n "${ALL_PKGS// }" ]]; then
    log "batch apt-install for selected cases:"
    log "  pkgs = $ALL_PKGS"
    APT_CMD="dpkg --configure -a 2>/dev/null;"
    APT_CMD+=" apt-get install -y -qq ${ALL_PKGS}; echo APT_EXIT=\$?"
    if ! "$CHECK_QEMU" "$APT_CMD" 2400; then
        log "WARN: batch apt install reported errors; per-case installs will retry"
    fi
    if [[ $HAS_KSELF -eq 1 ]]; then
        KSELF_LN='ln -sf /usr/kernel-selftest /usr/lib/kselftests 2>/dev/null;'
        "$CHECK_QEMU" "${KSELF_LN} ls /usr/lib/kselftests/run_kselftest.sh" 30
    fi
    if [[ $HAS_KMOD -eq 1 ]]; then
        log "re-injecting compiled modules after batch apt install"
        "$INJECT_MOD" hostshare /mnt/hostshare || log "WARN: re-inject after batch apt failed"
    fi
fi

while IFS=$'\t' read -r ID PKGS CMD TMOUT PASS; do
    [[ -z "${ID:-}" ]] && continue
    if ! want_run "$ID"; then continue; fi
    CMD="${RUNNER_DIR}/${CMD}"
    FAILRE="${DFLT_FAILRE}"

    log "============================================================"
    log " TEST CASE   : $ID"
    log " RUN COMMAND : $CMD"
    log " TIMEOUT     : ${TMOUT}s"
    log " PASS EXPECT : $PASS"
    log " FAIL EXPECT : $FAILRE"
    log " APT PKGS    : ${PKGS:-(none)}"
    log "============================================================"

    if [[ -n "${PKGS// }" && "$PKGS" != "_" ]]; then
        # Idempotent retry: pkgs were already installed in the batch step,
        # so this should return immediately.  But if batch install failed for
        # any reason, this is a real first-time install (perf/lmbench/ltp are
        # big), so give it a generous timeout.
        APT_CMD="dpkg --configure -a 2>/dev/null;"
        APT_CMD+=" apt-get install -y -qq ${PKGS}; echo APT_EXIT=\$?"
        if ! "$CHECK_QEMU" "$APT_CMD" 1800; then
            log "WARN: apt install for $ID failed; running test anyway"
        fi
        # compat: kernel-selftest pkg installs to /usr/kernel-selftest (yocto)
        # but runner expects /usr/lib/kselftests when kernel has no 'yocto' in name
        if [[ "$PKGS" == *kernel-selftest* ]]; then
            KSELF_LN='ln -sf /usr/kernel-selftest /usr/lib/kselftests 2>/dev/null;'
            "$CHECK_QEMU" "${KSELF_LN} ls /usr/lib/kselftests/run_kselftest.sh" 30
        fi
    fi

    if "$CHECK_QEMU" "$CMD" "$TMOUT" "$FAILRE" "$PASS"; then
        log "RESULT: $ID  PASSED"
        PASSED+=("$ID")
    else
        rc=$?
        log "RESULT: $ID  FAILED (rc=$rc)"
        FAILED+=("$ID")
    fi
done <<< "$CASE_TABLE"

# ---- summary ---------------------------------------------------------------
log "============================================================"
log "SUMMARY"
log "  PASSED  (${#PASSED[@]}): ${PASSED[*]:-}"
log "  FAILED  (${#FAILED[@]}): ${FAILED[*]:-}"
log "  SKIPPED (${#SKIPPED[@]}): ${SKIPPED[*]:-}"
log "============================================================"

if [[ ${#FAILED[@]} -gt 0 ]]; then
    exit 1
fi
exit 0
