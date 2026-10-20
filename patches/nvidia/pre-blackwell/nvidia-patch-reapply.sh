#!/bin/bash
# From omarchyvr (patches/nvidia/nvidia-patch-reapply.sh, MIT); PiBridge: the patches live in <version>/ (0001-0003).
# Re-apply the Pimax NVIDIA patches to the installed nvidia-open-dkms source tree, in order:
#   1. 0001-nvidia-2h1or-viewport.patch            (right eye shows the right half: PR #1262)
#   2. 0002-nvidia-2h1or-flip-fix.patch            (merged_head_sync: flips on 2Heads1OR; adds the module parameter)
#   3. 0003-nvidia-evo-makeroom-bounded-wait.patch (bounded wait in the EVO push-buffer make-room loop)
# (0004-nvidia-2h1or-vr-wider-than-5120.patch, untested, is never applied by this script.)
# Idempotent: a patch that is already in (patch -R --dry-run succeeds) is skipped.
#
# Why: the Pimax Dream Air needs 7104x3840 = "2 heads, 1 OR" on NVIDIA. Every nvidia-open-dkms upgrade replaces
# /usr/src/nvidia-<ver> with the pristine source, so the fixes have to be applied again BEFORE dkms builds the module.
# See linux/DISPLAY.md for the install commands (needs sudo).
#
# All patches are first applied to a scratch copy of the files they touch; the real tree is changed only if every
# patch applied (or was already in), so a conflict never leaves a half-patched tree.
#
# Untested drivers: only the versions in tested-versions.txt (next to this script) are patched. Any other driver is left
# alone (the stock module is built, the desktop keeps working) unless the user opted in: --allow-untested, or
# allow_untested=1 in /etc/omarchyvr/nvidia.conf (the app's switch "Allow the patch on untested NVIDIA drivers"; the
# system setup job writes the file). Opted in: a loud WARNING, patches applied with --fuzz=0 and, if that fails, --fuzz=2
# (said so); merged_head_sync is still required; --fallback still ends with a working stock module. Every run that
# patches or refuses writes /var/lib/omarchyvr/nvidia-patch-status (driver, result, fuzz) for the app.
#
# Usage: nvidia-patch-reapply.sh [--check] [--hook] [--rebuild] [--verify] [--version V] [--src-root DIR] [--patch FILE]...
#   --check       change nothing; exit 0 if all trees are patched, 2 if a tree needs a patch, 1 on conflict/errors
#   --hook        pacman hook mode: loud banner and a syslog line whenever the module would be built unpatched (exit 1)
#   --rebuild     after patching: dkms build + install --force for the running kernel, then rebuild the boot image
#                 (limine-mkinitcpio on Omarchy/Limine, else mkinitcpio -P), then --verify. The pacman hook does not
#                 need this: the dkms and initramfs hooks run after it.
#   --fallback    never leave the machine without a working module: if the patched dkms build fails (--rebuild),
#                 or (with --verify, the pacman hook after the dkms build) dkms has no installed module for the
#                 running kernel, put the stock files back (*.pimax-orig), build and install the stock module and
#                 the boot image, and report it loudly (exit 1). The desktop keeps working; the headset shows the
#                 left half in both eyes until the patches are fixed.
#   --force-build with --rebuild: build even when every patch was already in (the module lacked the fix)
#   --verify      only check the BUILT module: modinfo of nvidia-modeset must list the merged_head_sync parameter
#                 (exit 1 and a banner if not). Run after the dkms build (the 75 pacman hook does this).
#   --allow-untested  patch a driver that is not in tested-versions.txt (also allow_untested=1 in the conf file)
#   --version V   only /usr/src/nvidia-V (default: every /usr/src/nvidia-<digits> tree)
#   --src-root D  directory holding the nvidia-<ver> trees (default /usr/src, or $NVIDIA_SRC_ROOT)
#   --patch FILE  patch to use, repeatable, applied in the order given (default: the three above, see "Per-version
#                 patches"; with --patch the given files are used for every tree)
# Per-version patches: for the driver version V each of the three patches is taken from <this dir>/V/<name> when that
# file exists, else from <this dir>/610.57.04/<name> (the default set, written for the tested version). A version listed in
# ported-versions.txt has such a V/ directory: its patches were ported to that source, apply with --fuzz=0 and were
# compile-checked, but are NOT hardware-tested. It is still "untested" (needs the opt-in), but the opt-in uses the clean
# port and never the relaxed --fuzz=2 retry; the status file gets a line port=ported.
# Environment (tests): NVIDIA_PATCH_TESTED (tested list), NVIDIA_PATCH_PORTED (ported list), NVIDIA_PATCH_CONF (conf file),
# NVIDIA_PATCH_STATUS (status file).
# Exit codes: 0 ok (already patched or patched now), 1 error or conflict, 2 --check found an unpatched tree.
set -euo pipefail

HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
SRC_ROOT=${NVIDIA_SRC_ROOT:-/usr/src}
PATCH_FILES=()
TESTED_FILE=${NVIDIA_PATCH_TESTED:-$HERE/tested-versions.txt}
PORTED_FILE=${NVIDIA_PATCH_PORTED:-$HERE/ported-versions.txt}
DEFAULT_PATCH_NAMES=(0001-nvidia-2h1or-viewport.patch 0002-nvidia-2h1or-flip-fix.patch 0003-nvidia-evo-makeroom-bounded-wait.patch)
DEFAULT_SET=${NVIDIA_PATCH_DEFAULT_SET:-$HERE/610.57.04}
EXPLICIT_PATCHES=0
CONF_FILE=${NVIDIA_PATCH_CONF:-/etc/omarchyvr/nvidia.conf}
STATUS_FILE=${NVIDIA_PATCH_STATUS:-/var/lib/omarchyvr/nvidia-patch-status}
ALLOW_UNTESTED=0 FUZZ=0
CHECK=0 HOOK=0 REBUILD=0 VERIFY=0 FALLBACK=0 FORCE_BUILD=0 ONLY_VERSION=""
VIEWPORT_TARGET=src/nvidia-modeset/src/nvkms-evo.c
FLIP_MARKER_FILE=kernel-open/nvidia-modeset/nvidia-modeset-linux.c # carries the merged_head_sync parameter
BACKUP_SUFFIX=.pimax-orig
# the commands --rebuild / --verify run (overridable so that the test never touches the real ones)
DKMS=${NVIDIA_PATCH_DKMS:-dkms}
LIMINE_MKINITCPIO=${NVIDIA_PATCH_LIMINE:-limine-mkinitcpio}
MKINITCPIO=${NVIDIA_PATCH_MKINITCPIO:-mkinitcpio}
MODINFO=${NVIDIA_PATCH_MODINFO:-modinfo}

while [ $# -gt 0 ]; do
	case $1 in
	--check) CHECK=1 ;;
	--hook) HOOK=1 ;;
	--rebuild) REBUILD=1 ;;
	--verify) VERIFY=1 ;;
	--fallback) FALLBACK=1 ;;
	--allow-untested) ALLOW_UNTESTED=1 ;;
	--force-build) FORCE_BUILD=1 ;;
	--version) ONLY_VERSION=${2:?--version needs a value}; shift ;;
	--src-root) SRC_ROOT=${2:?--src-root needs a value}; shift ;;
	--patch) PATCH_FILES+=("${2:?--patch needs a value}"); EXPLICIT_PATCHES=1; shift ;;
	-h | --help) awk 'NR > 1 && /^#/ { sub(/^# ?/, ""); print; next } NR > 1 { exit }' "${BASH_SOURCE[0]}"; exit 0 ;;
	*) echo "unknown option $1 (try --help)" >&2; exit 1 ;;
	esac
	shift
done
if [ ${#PATCH_FILES[@]} -eq 0 ]; then
	PATCH_FILES=("${DEFAULT_PATCH_NAMES[@]/#/$DEFAULT_SET/}")
fi

# the opt-in saved by the app (parsed, never sourced)
if [ "$ALLOW_UNTESTED" = 0 ] && [ -r "$CONF_FILE" ] && grep -Eqi '^[[:space:]]*allow_untested[[:space:]]*=[[:space:]]*("?)(1|yes|true|on)\1[[:space:]]*$' "$CONF_FILE"; then
	ALLOW_UNTESTED=1
fi

# is_tested VERSION: listed in tested-versions.txt (built-in default when the file is missing)
is_tested() {
	if [ -r "$TESTED_FILE" ]; then
		sed 's/#.*//; s/[[:space:]]//g' "$TESTED_FILE" | grep -Fxq -- "$1"
	else
		[ "$1" = 610.57.04 ]
	fi
}

# is_ported VERSION: listed in ported-versions.txt (patches ported + compile-checked, not hardware-tested)
is_ported() {
	[ -r "$PORTED_FILE" ] && sed 's/#.*//; s/[[:space:]]//g' "$PORTED_FILE" | grep -Fxq -- "$1"
}

# select_patches VERSION: PATCH_FILES for that driver version (see "Per-version patches"); --patch files win
select_patches() {
	local n f
	[ "$EXPLICIT_PATCHES" = 1 ] && return 0
	PATCH_FILES=()
	for n in "${DEFAULT_PATCH_NAMES[@]}"; do
		f=$DEFAULT_SET/$n
		[ -f "$HERE/$1/$n" ] && f=$HERE/$1/$n
		[ -f "$f" ] || { echo "patch file $f not found" >&2; return 1; }
		PATCH_FILES+=("$(readlink -f -- "$f")")
	done
}

# write_status VERSION RESULT FUZZ: what the app shows (tested | untested-allowed | untested-refused | failed)
write_status() {
	[ "$CHECK" = 1 ] && return 0
	local old_fuzz="" port=""
	is_ported "$1" && port="port=ported"$'\n'
	if [ "$3" = - ] && [ -r "$STATUS_FILE" ] && grep -qx "driver=$1" "$STATUS_FILE"; then
		old_fuzz=$(sed -n 's/^fuzz=//p' "$STATUS_FILE" | head -n1) # nothing applied this run: keep what the last patch run used
	fi
	{
		mkdir -p "$(dirname "$STATUS_FILE")" &&
			printf 'driver=%s\nresult=%s\nfuzz=%s\ntime=%s\n%s' "$1" "$2" "${old_fuzz:-$3}" "$(date -u +%FT%TZ)" "$port" >"$STATUS_FILE.new" &&
			mv -f "$STATUS_FILE.new" "$STATUS_FILE" && chmod 644 "$STATUS_FILE"
	} 2>/dev/null || true
}

say() { echo "==> nvidia-patch-reapply: $*"; }

banner() { # banner WHAT REASON
	{
		echo "################################################################################"
		echo "# PIMAX: $1"
		echo "# ($2)"
		echo "# The nvidia module that dkms builds next is NOT fully patched: the Dream Air will show the"
		echo "# left half in both eyes and/or hang the display engine (Xid / \"Idling display engine\")."
		echo "# Check whether NVIDIA fixed this upstream (then nothing is needed) or regenerate the patches in"
		echo "# patches/nvidia/ of omarchyvr for the new source."
		echo "################################################################################"
	} >&2
	if [ "$HOOK" = 1 ] && [ -z "${NVIDIA_PATCH_NO_SYSLOG:-}" ] && command -v logger >/dev/null 2>&1; then
		logger -t pimax-nvidia-patch -p user.err "$1 ($2)" || true
	fi
}

tested_list() { if [ -r "$TESTED_FILE" ]; then sed 's/#.*//; s/[[:space:]]//g; /^$/d' "$TESTED_FILE" | paste -sd' '; else echo 610.57.04; fi; }
# untested driver, no opt-in: nothing is patched, the stock module is built, pacman is not blocked
refuse_untested() { # refuse_untested DIR VERSION
	{
		echo "################################################################################"
		echo "# PIMAX: NVIDIA driver $2 is not tested with the omarchyvr NVIDIA patches."
		echo "# The patches were NOT applied; the stock nvidia module is built and the desktop works."
		echo "# The Dream Air headset needs the patches: turn on 'Allow the patch on untested NVIDIA drivers'"
		echo "# in the omarchyvr app (Onboarding > System setup), then press Apply. Tested: $(tested_list)."
		is_ported "$2" && echo "# Patches ported to $2 exist (compile-checked, not hardware-tested): the opt-in applies those."
		echo "################################################################################"
	} >&2
	if [ "$HOOK" = 1 ] && [ -z "${NVIDIA_PATCH_NO_SYSLOG:-}" ] && command -v logger >/dev/null 2>&1; then
		logger -t pimax-nvidia-patch -p user.warning "NVIDIA driver $2 is not tested; patches not applied (allow it in the omarchyvr app)" || true
	fi
}
warn_untested() { # warn_untested DIR VERSION
	{
		echo "################################################################################"
		echo "# PIMAX WARNING: NVIDIA driver $2 is NOT tested with the omarchyvr NVIDIA patches."
		echo "# You allowed patching untested drivers, so they are applied anyway. The patched module may fail to"
		echo "# build, or the headset picture may break. If the build fails, the stock module is installed instead"
		echo "# (--fallback) and the desktop keeps working."
		is_ported "$2" && echo "# $2 has ported patches (compile-checked, NOT hardware-tested): see docs/NVIDIA-615.md for the test plan."
		echo "################################################################################"
	} >&2
}

# --verify: the built module must carry the flip-fix parameter
verify_module() {
	local kver out
	kver=$(uname -r)
	if ! out=$("$MODINFO" -k "$kver" nvidia-modeset 2>&1); then
		banner "cannot verify the nvidia-modeset module for $kver" "modinfo failed: ${out:0:200}"
		return 1
	fi
	if printf '%s\n' "$out" | grep -q 'merged_head_sync'; then
		say "verified: nvidia-modeset for $kver has the merged_head_sync parameter"
	else
		banner "nvidia-modeset for $kver has NO merged_head_sync parameter" "the module was built without 0002-nvidia-2h1or-flip-fix.patch"
		return 1
	fi
}

# restore_stock DIR: the *.pimax-orig files back in place (the stock source), then dkms build + install for the running
# kernel and the boot image. Used by --fallback only.
restore_stock() {
	local d=$1 ver o kver
	ver=${d##*/nvidia-}
	kver=$(uname -r)
	while IFS= read -r o; do
		mv -f -- "$o" "${o%"$BACKUP_SUFFIX"}"
	done < <(find "$d" -name "*$BACKUP_SUFFIX" -type f)
	say "fallback: stock source restored in $d; building the stock module for $kver"
	"$DKMS" build "nvidia/$ver" -k "$kver" --force && "$DKMS" install "nvidia/$ver" -k "$kver" --force || {
		banner "the STOCK nvidia module could not be built either" "dkms build/install nvidia/$ver failed"
		return 1
	}
	if command -v "$LIMINE_MKINITCPIO" >/dev/null 2>&1; then "$LIMINE_MKINITCPIO"; else "$MKINITCPIO" -P; fi
	write_status "$ver" failed "$FUZZ"
	banner "the patched nvidia module failed to build: the STOCK module is installed instead" "the desktop works; the headset needs the patches (see the pacman log)"
}
module_installed() { "$DKMS" status "nvidia/$1" -k "$(uname -r)" 2>/dev/null | grep -q 'installed'; }

if [ "$VERIFY" = 1 ]; then
	if [ "$FALLBACK" = 1 ]; then
		for d in "$SRC_ROOT"/nvidia-[0-9]*; do
			[ -d "$d" ] || continue
			case $d in *.orig | *.pimax-orig) continue ;; esac
			v=${d##*/nvidia-}
			if ! module_installed "$v" && [ -n "$(find "$d" -name "*$BACKUP_SUFFIX" -type f -print -quit)" ]; then
				restore_stock "$d" || true
				exit 1
			fi
		done
	fi
	if [ -r "$STATUS_FILE" ] && grep -qx 'result=untested-refused' "$STATUS_FILE"; then
		say "verify: NVIDIA driver $(sed -n 's/^driver=//p' "$STATUS_FILE") is untested and was not patched (not allowed in the app); nothing to verify"
		exit 0
	fi
	verify_module
	exit $?
fi

command -v patch >/dev/null 2>&1 || { echo "patch(1) is not installed (pacman -S patch)" >&2; exit 1; }
# patch(1) is run with -d <dir>, which changes directory before it opens the -i file: make the paths absolute
for i in "${!PATCH_FILES[@]}"; do
	[ -f "${PATCH_FILES[$i]}" ] || { echo "patch file ${PATCH_FILES[$i]} not found" >&2; exit 1; }
	PATCH_FILES[$i]=$(readlink -f -- "${PATCH_FILES[$i]}")
done

# One run at a time: the pacman hook and a manual --rebuild must not both see "unpatched" and then fight over the
# backup files. The lock lives on the first patch file (readable by everybody, so --check works without root).
if command -v flock >/dev/null 2>&1; then
	exec 9<"${PATCH_FILES[0]}"
	flock 9
fi

# files touched by a patch (a/ b/ stripped)
patch_files_of() { sed -n 's|^+++ b/||p' "$1" | sed 's/\t.*//'; }

SCRATCH=$(mktemp -d "${TMPDIR:-/tmp}/nvpatch-scratch.XXXXXX")
trap 'rm -rf "$SCRATCH"' EXIT

pdry() { patch -p1 --dry-run --fuzz=$FUZZ --no-backup-if-mismatch -s -d "$1" -i "$2" >/dev/null 2>&1; }       # forward applies cleanly?
pdry_r() { patch -R -p1 --dry-run --fuzz=$FUZZ --no-backup-if-mismatch -s -d "$1" -i "$2" >/dev/null 2>&1; }  # already in?

# Plan the tree in a scratch copy. Sets PLAN_STATE[i] (patched|apply|fixed-upstream), PLAN_FILES (files to copy over),
# PLAN_ERR (reason, when it returns 1). Returns 0 ok, 1 conflict, 2 not a tree we can handle (missing target).
plan_tree() {
	local dir=$1 s=$SCRATCH/plan n=${#PATCH_FILES[@]} i f k p
	rm -rf "$s"
	mkdir -p "$s"
	PLAN_STATE=() PLAN_FILES=() PLAN_ERR=""
	[ -f "$dir/$VIEWPORT_TARGET" ] || return 2
	local all=()
	for p in "${PATCH_FILES[@]}"; do
		while IFS= read -r f; do all+=("$f"); done < <(patch_files_of "$p")
	done
	for f in $(printf '%s\n' "${all[@]}" | sort -u); do
		if [ -f "$dir/$f" ]; then
			mkdir -p "$s/$(dirname "$f")"
			cp -p "$dir/$f" "$s/$f"
		fi
	done
	# peel the trailing patches that are already in (a later patch may touch the same lines as an earlier one)
	k=$n
	while [ $k -gt 0 ] && pdry_r "$s" "${PATCH_FILES[$((k - 1))]}"; do
		patch -R -p1 --fuzz=$FUZZ --no-backup-if-mismatch -s -d "$s" -i "${PATCH_FILES[$((k - 1))]}" >/dev/null
		k=$((k - 1))
	done
	for ((i = k; i < n; i++)); do PLAN_STATE[$i]=patched; done
	for ((i = 0; i < k; i++)); do
		p=${PATCH_FILES[$i]}
		if pdry "$s" "$p"; then
			patch -p1 --fuzz=$FUZZ --no-backup-if-mismatch -s -d "$s" -i "$p" >/dev/null
			PLAN_STATE[$i]=apply
		elif pdry_r "$s" "$p"; then
			PLAN_STATE[$i]=patched
		elif [ "${p##*/}" = 0001-nvidia-2h1or-viewport.patch ] &&
			awk '/^void nvSetViewPortsEvo/,/^}/' "$s/$VIEWPORT_TARGET" | tr '\n' ' ' | grep -Eq 'EvoSetViewportPointIn\([^;]*mergeHeadSection'; then
			# the call (it may span two lines) no longer passes a constant x offset: upstream, or somebody, fixed it another way
			PLAN_STATE[$i]=fixed-upstream
		else
			PLAN_ERR="${p##*/} does not apply: the code it patches changed"
			return 1
		fi
	done
	# re-apply the peeled patches on top (they were in before, so this works unless an earlier patch was missing)
	for ((i = k; i < n; i++)); do
		if ! patch -p1 --fuzz=$FUZZ --no-backup-if-mismatch -s -d "$s" -i "${PATCH_FILES[$i]}" >/dev/null 2>&1; then
			PLAN_ERR="${PATCH_FILES[$i]##*/} was in, but no longer fits on top of the patches before it"
			return 1
		fi
	done
	# which real files change
	while IFS= read -r f; do
		[ -f "$s/$f" ] && ! cmp -s "$s/$f" "$dir/$f" && PLAN_FILES+=("$f")
	done < <(cd "$s" && find . -type f | sed 's|^\./||' | sort)
	return 0
}

trees=()
shopt -s nullglob
for d in "$SRC_ROOT"/nvidia-[0-9]*; do
	[ -d "$d" ] || continue
	case $d in *.orig | *.pimax-orig) continue ;; esac
	ver=${d##*/nvidia-}
	[ -z "$ONLY_VERSION" ] || [ "$ver" = "$ONLY_VERSION" ] || continue
	trees+=("$d")
done
shopt -u nullglob
if [ ${#trees[@]} -eq 0 ]; then
	if [ "$HOOK" = 1 ]; then
		# the hook runs because nvidia-open-dkms was just installed: a missing tree means the dkms build has nothing to patch
		banner "the Pimax NVIDIA patches could NOT be applied to $SRC_ROOT/nvidia-<version>" "no nvidia source tree found${ONLY_VERSION:+ for version $ONLY_VERSION}"
		exit 1
	fi
	say "no nvidia-<version> source tree in $SRC_ROOT${ONLY_VERSION:+ for version $ONLY_VERSION}: nothing to do"
	exit 0
fi

rc=0
patched_now=()
for d in "${trees[@]}"; do
	ver=${d##*/nvidia-}
	UNTESTED=0 FUZZ=0
	if ! is_tested "$ver"; then
		UNTESTED=1
		if [ "$ALLOW_UNTESTED" = 0 ]; then
			if [ -f "$d/$VIEWPORT_TARGET" ] || [ -f "$d/dkms.conf" ]; then
				refuse_untested "$d" "$ver"
				write_status "$ver" untested-refused -
			else
				say "$d: $VIEWPORT_TARGET not found, skipped"
			fi
			continue
		fi
		warn_untested "$d" "$ver"
	fi
	okres=tested
	[ $UNTESTED = 1 ] && okres=untested-allowed
	select_patches "$ver" || { rc=1; continue; }
	PORTED=0
	is_ported "$ver" && [ -f "$HERE/$ver/${DEFAULT_PATCH_NAMES[0]}" ] && PORTED=1
	[ $PORTED = 1 ] && say "$d: using the patches ported to $ver ($HERE/$ver/): they were compile-checked, NOT tested on hardware"
	plan_rc=0
	plan_tree "$d" || plan_rc=$?
	# a ported version applies with --fuzz=0 or not at all: a relaxed match there means the source changed again
	if [ $plan_rc = 1 ] && [ $UNTESTED = 1 ] && [ $PORTED = 0 ]; then
		say "$d: the patches do not apply with --fuzz=0 ($PLAN_ERR); trying --fuzz=2 (untested driver)"
		FUZZ=2
		plan_rc=0
		plan_tree "$d" || plan_rc=$?
		if [ $plan_rc = 0 ]; then
			echo "==> nvidia-patch-reapply: WARNING: $d: the patches apply only with --fuzz=2 (some context lines differ). Check the headset picture after the build." >&2
		fi
	fi
	if [ $plan_rc = 2 ]; then
		if [ "$HOOK" = 1 ] && [ -f "$d/dkms.conf" ]; then
			# a real dkms tree whose layout changed: dkms would build it unpatched
			banner "the Pimax NVIDIA patches could NOT be applied to $d" "$VIEWPORT_TARGET not found in a dkms source tree"
			rc=1
		else
			say "$d: $VIEWPORT_TARGET not found, skipped"
		fi
		continue
	elif [ $plan_rc = 1 ]; then
		banner "the Pimax NVIDIA patches could NOT be applied to $d" "$PLAN_ERR$([ $FUZZ = 2 ] && echo '; also tried --fuzz=2')"
		write_status "$ver" failed "$FUZZ"
		rc=1
		continue
	fi
	todo=()
	for i in "${!PATCH_FILES[@]}"; do
		nm=${PATCH_FILES[$i]##*/}
		case ${PLAN_STATE[$i]} in
		patched) say "$d: $nm already patched" ;;
		fixed-upstream) say "$d: $nm: the source already passes a per-head x offset to EvoSetViewportPointIn; not patching. Verify with vk_display_test, then retire this patch." ;;
		apply) todo+=("$nm") ;;
		esac
	done
	if [ ${#todo[@]} -eq 0 ]; then
		say "$d: already patched"
		write_status "$ver" "$okres" -
		continue
	fi
	if [ "$CHECK" = 1 ]; then
		say "$d: NOT patched, needs: ${todo[*]} (they apply $([ $FUZZ = 2 ] && echo 'only with --fuzz=2' || echo cleanly))"
		[ $rc -ne 1 ] && rc=2
		continue
	fi
	# the files are replaced in place and a .pimax-orig backup is created next to each
	nowrite=0
	for f in "${PLAN_FILES[@]}"; do
		[ -w "$d/$f" ] && [ -w "$d/$(dirname "$f")" ] || nowrite=1  # the directory too: the backup is created next to the file
	done
	if [ $nowrite = 1 ]; then
		echo "$d (a file to patch) is not writable: run as root (sudo)" >&2
		rc=1
		continue
	fi
	for f in "${PLAN_FILES[@]}"; do
		[ -e "$d/$f$BACKUP_SUFFIX" ] || cp -p "$d/$f" "$d/$f$BACKUP_SUFFIX"
		cp "$SCRATCH/plan/$f" "$d/$f"
	done
	# the real tree must now equal the plan, and carry the flip-fix parameter if that patch is in the list
	again=0
	plan_tree "$d" || again=1
	if [ $again = 0 ]; then
		for i in "${!PATCH_FILES[@]}"; do [ "${PLAN_STATE[$i]}" = apply ] && again=1; done
	fi
	if [ $again = 1 ]; then
		banner "the Pimax NVIDIA patches could NOT be applied to $d" "the files do not look patched after copying the result"
		write_status "$ver" failed "$FUZZ"
		rc=1
		continue
	fi
	for p in "${PATCH_FILES[@]}"; do
		if [ "${p##*/}" = 0002-nvidia-2h1or-flip-fix.patch ] && ! grep -q merged_head_sync "$d/$FLIP_MARKER_FILE" 2>/dev/null; then
			banner "the Pimax NVIDIA patches could NOT be applied to $d" "merged_head_sync is missing from $FLIP_MARKER_FILE after patching"
			write_status "$ver" failed "$FUZZ"
			rc=1
			continue 2
		fi
	done
	say "$d: patched ${todo[*]} (originals kept as <file>$BACKUP_SUFFIX)$([ $FUZZ = 2 ] && echo ' with --fuzz=2')"
	write_status "$ver" "$okres" "$FUZZ"
	patched_now+=("$d")
done

if [ "$REBUILD" = 1 ] && [ "$FORCE_BUILD" = 1 ] && [ "${#patched_now[@]}" -eq 0 ]; then
	patched_now=("${trees[@]}")
fi
if [ "$REBUILD" = 1 ] && [ "${#patched_now[@]}" -gt 0 ] && [ "$CHECK" = 0 ]; then
	for d in "${patched_now[@]}"; do
		ver=${d##*/nvidia-}
		say "dkms build + install nvidia/$ver for $(uname -r)"
		if ! { "$DKMS" build "nvidia/$ver" -k "$(uname -r)" --force && "$DKMS" install "nvidia/$ver" -k "$(uname -r)" --force; }; then
			if [ "$FALLBACK" = 1 ]; then
				restore_stock "$d" || true
				exit 1
			fi
			write_status "$ver" failed "$FUZZ"
			banner "dkms could not build the patched nvidia module" "see the dkms output above"
			exit 1
		fi
	done
	if command -v "$LIMINE_MKINITCPIO" >/dev/null 2>&1; then
		# Omarchy/Limine boots a UKI that contains the nvidia modules; /etc/mkinitcpio.d has no presets for mkinitcpio -P
		say "limine-mkinitcpio (rebuilds the UKI with the new nvidia modules)"
		"$LIMINE_MKINITCPIO"
	else
		say "mkinitcpio -P (the nvidia modules load from the initramfs)"
		"$MKINITCPIO" -P
	fi
	verify_module || rc=1
	say "done: reboot to load the patched nvidia modules"
fi
exit $rc
