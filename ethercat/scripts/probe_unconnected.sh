#!/usr/bin/env bash
# Temporary real-kernel module test with NO connected slaves. Always unload
# the modules created here. Does not install services or alter NIC settings.
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
igh_root="$PWD"
if [[ $# -ne 1 || ! "$1" =~ ^[a-zA-Z0-9_.:-]+$ ]]; then
    printf 'Usage: sudo bash scripts/probe_unconnected.sh INTERFACE\n' >&2
    exit 2
fi
igh_interface="$1"
igh_net="/sys/class/net/$igh_interface"
[[ "$EUID" -eq 0 ]] || { printf 'Module loading requires root.\n' >&2; exit 2; }
test -d "$igh_net/device"
test ! -d "$igh_net/wireless"
test "$(cat "$igh_net/carrier")" = 0 || {
    printf 'This test requires the EtherCAT cable to be disconnected.\n' >&2; exit 2;
}
test ! -e /sys/module/ec_master
test ! -e /sys/module/ec_generic
igh_mac="$(cat "$igh_net/address")"
[[ "$igh_mac" =~ ^([0-9a-fA-F]{2}:){5}[0-9a-fA-F]{2}$ ]]
[[ "$igh_mac" != '00:00:00:00:00:00' && "$igh_mac" != 'ff:ff:ff:ff:ff:ff' ]]
igh_master="$igh_root/build/igh-source/master/ec_master.ko"
igh_generic="$igh_root/build/igh-source/devices/ec_generic.ko"
igh_cli="$igh_root/build/igh-install/bin/ethercat"
for igh_module in "$igh_master" "$igh_generic"; do
    igh_vermagic="$(modinfo -F vermagic "$igh_module")"
    test "${igh_vermagic%% *}" = "$(uname -r)"
done
test -x "$igh_cli"

igh_master_loaded=0
igh_generic_loaded=0
cleanup() {
    local result=$?
    trap - EXIT
    if (( igh_generic_loaded )); then rmmod ec_generic || result=1; fi
    if (( igh_master_loaded )); then rmmod ec_master || result=1; fi
    if (( igh_master_loaded )); then
        if [[ ! -e /sys/module/ec_master && ! -e /sys/module/ec_generic ]]; then
            printf 'Cleanup: temporary EtherCAT modules unloaded.\n'
        else
            printf 'Cleanup incomplete: check lsmod before another test.\n' >&2
            result=1
        fi
    fi
    exit "$result"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

printf 'Kernel: %s\nInterface: %s (%s), carrier=0\n' "$(uname -r)" "$igh_interface" "$igh_mac"
insmod "$igh_master" "main_devices=$igh_mac"
igh_master_loaded=1
udevadm settle --timeout=5
test -c /dev/EtherCAT0
printf 'Master-only probe:\n'
"$igh_cli" --master 0 master

test "$(cat "$igh_net/carrier")" = 0
insmod "$igh_generic"
igh_generic_loaded=1
printf 'Generic-driver probe:\n'
igh_status="$("$igh_cli" --master 0 master)"
printf '%s\n' "$igh_status"
[[ "$igh_status" == *"Phase: Idle"* && "$igh_status" == *"Active: no"* \
   && "$igh_status" == *"Main: $igh_mac (attached)"* ]] || {
    printf 'Generic driver did not attach the expected inactive master.\n' >&2; exit 1;
}
igh_slaves="$("$igh_cli" --master 0 slaves)"
test -z "$igh_slaves" || {
    printf 'Unexpected slaves detected; stopping this disconnected test.\n' >&2; exit 1;
}
test "$(cat "$igh_net/carrier")" = 0
printf 'PASS: real master/device API and generic driver loaded; zero slaves; no motion commands.\n'
