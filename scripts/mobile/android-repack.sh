#!/usr/bin/env bash
# Copyright 2020 The Defold Foundation
# Licensed under the Defold License version 1.0 (the "License"); you may not use
# this file except in compliance with the License.
#
# You may obtain a copy of the License, together with FAQs at
# https://www.defold.com/license
#
# Unless required by applicable law or agreed to in writing, software distributed
# under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
# CONDITIONS OF ANY KIND, either express or implied. See the License for the
# specific language governing permissions and limitations under the License.


# Mathias Westerdahl, Jakob Pogulis
set -eu

SCRIPT_NAME="$(basename "${0}")"
SCRIPT_PATH="$(cd "$(dirname "${0}")"; pwd)"


# ----------------------------------------------------------------------------
# Script functions
# ----------------------------------------------------------------------------
function terminate() {
    echo "TERM - ${1:-}" && exit ${2:-1}
}

function terminate_usage() {
    echo "Usage: ${SCRIPT_NAME} <source> [<certificate>, <key>]"
    echo "  source       - absolute filepath to the source apk to repack"
    echo "  certificate  - (optional) absolute filepath to the certificate file (pem)"
    echo "  key          - (optional) absolute filepath to the keyfile (pk8)"
    exit 1
}

function terminate_trap() {
    trap - SIGHUP SIGINT SIGTERM
    find "${ROOT}"
    [ ! -z "${ROOT:-}" ] && rm -rf "${ROOT}"
    terminate "An unexpected error occurred."
}


# ----------------------------------------------------------------------------
# Script environment
# ----------------------------------------------------------------------------
[ ! -z "${DYNAMO_HOME:-}" ] || terminate "DYNAMO_HOME is not set"
DEFOLD_HOME="$(cd "${DYNAMO_HOME}/../.."; pwd)"

ANDROID_NDK_VERSION=20
ANDROID_NDK_ROOT="${DYNAMO_HOME}/ext/SDKs/android-ndk-r${ANDROID_NDK_VERSION}"

[ -d "${ANDROID_NDK_ROOT}" ] || terminate "ANDROID_NDK_ROOT=${ANDROID_NDK_ROOT} does not exist"

SOURCE="${1:-}" && [ ! -z "${SOURCE}" ] || terminate_usage
SOURCE="$(cd "$(dirname "${SOURCE}")"; pwd)/$(basename "${SOURCE}")"
CERTIFICATE="${2:-}"
KEYFILE="${3:-}"

ZIP="zip"
UNZIP="unzip"
ZIPALIGN="${DEFOLD_HOME}/com.dynamo.cr/com.dynamo.cr.bob/libexec/x86_64-darwin/zipalign"
APKC="${DYNAMO_HOME}/ext/bin/x86_64-darwin/apkc"
GDBSERVER=${ANDROID_NDK_ROOT}/prebuilt/android-arm/gdbserver/gdbserver

ENGINE_LIB="${DYNAMO_HOME}/bin/armv7-android/libdmengine.so"
ENGINE_64_LIB="${DYNAMO_HOME}/bin/arm64-android/libdmengine.so"
ENGINE_DEX="${DYNAMO_HOME}/share/java/classes.dex"

[ $(which "${ZIP}") ] || terminate "'${ZIP}' is not installed"
[ $(which "${UNZIP}") ] || terminate "'${UNZIP}' is not installed"
[ $(which "${ZIPALIGN}") ] || terminate "'${ZIPALIGN}' is not installed"
[ $(which "${APKC}") ] || terminate "'${APKC}' is not installed"

[ -f "${ENGINE_LIB}" ] || echo "Engine does not exist: ${ENGINE_LIB}"
[ -f "${ENGINE_64_LIB}" ] || echo "Engine does not exist: ${ENGINE_64_LIB}"
[ -f "${SOURCE}" ] || terminate "Source does not exist: ${SOURCE}"
[ -f "${ENGINE_DEX}" ] || terminate "Engine does not exist: ${ENGINE_DEX}"
if [ ! -z "${CERTIFICATE}" ] || [ ! -z "${KEYFILE}" ]; then
    [ ! -z "${KEYFILE}" ] || terminate "Keyfile required if certificate is specified."
    [ -f "${CERTIFICATE}" ] || terminate "Certificate does not exist: ${CERTIFICATE}"
    [ -f "${KEYFILE}" ] || terminate "Keyfile does not exist: ${KEYFILE}"

    CERTIFICATE="$(cd "$(dirname "${CERTIFICATE}")"; pwd)/$(basename "${CERTIFICATE}")"
    KEYFILE="$(cd "$(dirname "${KEYFILE}")"; pwd)/$(basename "${KEYFILE}")"
fi


# ----------------------------------------------------------------------------
# Script
# ----------------------------------------------------------------------------
ROOT="$(mktemp -d)"
trap 'terminate_trap' SIGHUP SIGINT SIGTERM EXIT

BUILD="${ROOT}/build"
REPACKZIP="${ROOT}/repack.zip"
REPACKZIP_ALIGNED="${ROOT}/repack.aligned.zip"

APPLICATION="$(basename "${SOURCE}" ".apk")"
TARGET="$(cd "$(dirname "${SOURCE}")"; pwd)/${APPLICATION}.repack"

"${UNZIP}" "${SOURCE}" -d "${BUILD}" > /dev/null 2>&1
(
    cd "${BUILD}"

    if [ -d "lib/armeabi-v7a" ]; then
        EXENAME=`(cd lib/armeabi-v7a && ls lib*.so)`
    fi
    if [ -d "lib/arm64-v8a" ]; then
        EXENAME_64=`(cd lib/arm64-v8a && ls lib*.so)`
    fi

    rm -rf "META-INF"
    if [ -e "${ENGINE_LIB}" ]; then
        cp -v "${ENGINE_LIB}" "lib/armeabi-v7a/${EXENAME}"
    fi
    if [ -e "${ENGINE_64_LIB}" ]; then
        cp -v "${ENGINE_64_LIB}" "lib/arm64-v8a/${EXENAME_64}"
    fi
    cp -v "${ENGINE_DEX}" "classes.dex"

    if [ -e "$GDBSERVER" ]; then
        cp -v "${ANDROID_NDK_ROOT}/prebuilt/android-arm/gdbserver/gdbserver" ./lib/armeabi-v7a/gdbserver
    fi

    ${ZIP} -qr "${REPACKZIP}" "."
)

"${ZIPALIGN}" -v 4 "${REPACKZIP}" "${REPACKZIP_ALIGNED}" > /dev/null 2>&1

if [ ! -z "${CERTIFICATE}" ]; then
    "${APKC}" --in="${REPACKZIP_ALIGNED}" --out="${TARGET}.apk" --cert="${CERTIFICATE}" --key="${KEYFILE}" > /dev/null 2>&1
else
    "${APKC}" --in="${REPACKZIP_ALIGNED}" --out="${TARGET}.apk" > /dev/null 2>&1
fi

rm -rf "${ROOT}"


# ----------------------------------------------------------------------------
# Script teardown
# ----------------------------------------------------------------------------
trap - SIGHUP SIGINT SIGTERM EXIT
echo "Wrote ${TARGET}.apk"
exit 0
