#!/usr/bin/env bash
# Build and install into this project's build/ only. No sudo, modprobe,
# modules_install, service start, NIC binding or master requests.
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
igh_root="$PWD"
igh_prefix="$igh_root/build/igh-install"
igh_kernel="${IGH_KERNEL_RELEASE:-$(uname -r)}"
igh_jobs="${IGH_BUILD_JOBS:-4}"
mkdir -p build
trap 'printf "Build failed; inspect %s/build/*log\n" "$igh_root" >&2' ERR

for tool in gcc g++ make git cmake ninja autoreconf libtoolize pkg-config python3; do
    command -v "$tool" >/dev/null || { printf 'Missing build tool: %s\n' "$tool" >&2; exit 1; }
done
pkg-config --exists yaml-0.1 zlib
test -f "/lib/modules/$igh_kernel/build/Makefile"

checkout_source() {
    local name="$1" directory="$2" url revision
    url="$(python3 -c 'import json,sys; print(json.load(open("sources.json"))[sys.argv[1]]["url"])' "$name")"
    revision="$(python3 -c 'import json,sys; print(json.load(open("sources.json"))[sys.argv[1]]["revision"])' "$name")"
    if test ! -d "$directory/.git"; then
        test ! -e "$directory" || { printf 'Source path already exists: %s\n' "$directory" >&2; return 1; }
        git init -q "$directory"
        git -C "$directory" remote add origin "$url"
        git -C "$directory" fetch -q --depth 1 origin "$revision"
        git -C "$directory" checkout -q --detach FETCH_HEAD
    fi
    test "$(git -C "$directory" rev-parse HEAD)" = "$revision" || {
        printf 'Source revision mismatch: %s; review before rebuilding\n' "$directory" >&2; return 1;
    }
}
checkout_source ethercat build/igh-source
checkout_source rtipc build/rtipc-source

while IFS= read -r igh_patch; do
    igh_patch="$igh_root/$igh_patch"
    if git -C build/rtipc-source apply --check "$igh_patch" 2>/dev/null; then
        git -C build/rtipc-source apply "$igh_patch"
    else
        git -C build/rtipc-source apply --reverse --check "$igh_patch"
    fi
done < <(python3 -c 'import json; print("\n".join(json.load(open("sources.json"))["rtipc"]["patches"]))')

cmake -S build/rtipc-source -B build/rtipc -G Ninja \
    -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$igh_prefix" \
    -DCMAKE_INSTALL_LIBDIR=lib > build/rtipc-configure.log 2>&1
cmake --build build/rtipc -j "$igh_jobs" > build/rtipc-build.log 2>&1
cmake --install build/rtipc > build/rtipc-install.log 2>&1
(
    cd build/igh-source
    if test ! -x configure; then bash bootstrap > ../igh-bootstrap.log 2>&1; fi
    PKG_CONFIG_PATH="$igh_prefix/lib/pkgconfig${PKG_CONFIG_PATH:+:$PKG_CONFIG_PATH}" \
        ./configure --prefix="$igh_prefix" --sysconfdir="$igh_prefix/etc" \
        --with-linux-dir="/lib/modules/$igh_kernel/build" \
        --enable-generic --disable-8139too --disable-eoe --enable-fakeuserlib \
        --disable-initd --with-systemdsystemunitdir=no > ../igh-configure.log 2>&1
    make -j "$igh_jobs" all modules > ../igh-build.log 2>&1
    make install > ../igh-install.log 2>&1
)
printf '%s\n' "$igh_kernel" > build/kernel-release.txt
"$igh_prefix/bin/ethercat" version
printf 'Built real/fake libraries, CLI and kernel modules for %s\n' "$igh_kernel"
printf 'Local prefix: %s\nNo kernel module loaded or network interface bound.\n' "$igh_prefix"
