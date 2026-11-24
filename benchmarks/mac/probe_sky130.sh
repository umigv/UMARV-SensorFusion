#!/usr/bin/env bash
# Download-only dependency checks, with all writes confined to a temporary directory.
set -u
scratch=$(mktemp -d)
trap 'rm -rf "$scratch"' EXIT
run() {
    printf '\n$'
    printf ' %q' "$@"
    printf '\n'
    "$@"
    printf '[exit %d]\n' "$?"
}
for tool in yosys yosys-abc sta openroad OpenROAD openlane openlane2 docker podman nix nix-shell volare tt_tool pip; do
    run command -v "$tool"
done
for var in PDK_ROOT PDK OPENLANE_ROOT OPENROAD_ROOT SKY130_LIBERTY; do
    printf '%s=%s\n' "$var" "${!var-}"
done
roots=(/opt /usr/local /usr/share /var/cache/apt/archives "$HOME" /tmp)
if [[ -n "${PDK_ROOT-}" && -d "$PDK_ROOT" ]]; then roots+=("$PDK_ROOT"); fi
find "${roots[@]}" -type f \( -name yosys -o -name sta -o -name openroad -o -name openlane -o -name tt_tool.py -o -iname '*.lib' -o -iname '*.lib.gz' -o -iname '*.tlef' -o -iname '*.lef' -o -iname '*yosys*.deb' \) -print 2>/dev/null
cd "$scratch" || exit 1
for pkg in yosys opensta openroad; do
    run apt-get -o Acquire::Retries=0 -o Acquire::http::Timeout=5 download "$pkg"
done
run curl -fsSL --connect-timeout 5 --max-time 10 -o oss-release.json https://api.github.com/repos/YosysHQ/oss-cad-suite-build/releases/latest
run curl -fsSL --connect-timeout 5 --max-time 10 -o pdks-release.json https://api.github.com/repos/efabless/volare/releases
run curl -fsSL --connect-timeout 5 --max-time 10 -o openroad-release.json https://api.github.com/repos/The-OpenROAD-Project/OpenROAD/releases/latest
run curl -fsSL --connect-timeout 5 --max-time 10 -o yowasp.json https://pypi.org/pypi/yowasp-yosys/json
