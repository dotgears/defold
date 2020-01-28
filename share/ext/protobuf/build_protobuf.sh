#!/usr/bin/env bash

readonly PRODUCT=protobuf
readonly VERSION=2.3.0
readonly BASE_URL=https://github.com/mathiaswking/protobuf-${VERSION}/archive
readonly FILE_URL=protobuf-${VERSION}.tar.gz

readonly CONFIGURE_ARGS="--with-protoc=../cross_tmp/src/protoc --disable-shared"

#test the compiler
if [ "$(uname -o)" == "Msys" ]; then
	WINDEFINES=$(echo | g++ -E -dM - | grep WIN32)
	if [ -z "$WINDEFINES" ]; then
		echo "Compiler doesn't support WIN32 defines!"
		echo G++=$(which g++)
		echo $(g++ --version)
		exit 1
	fi
fi


. ../common.sh

# The package is ancient
# A fairly small package of ~1.5MB, so it's stored locally
function download() {
    mkdir -p ../download
    cp ./${FILE_URL} ../download/
}

download

#
# Build protoc locally first,
# in order to make the cross platform configuration work
#
rm -rf cross_tmp
mkdir cross_tmp
pushd cross_tmp >/dev/null
tar xfz ../../download/$FILE_URL --strip-components=1

# We need to apply patches for this cross_tmp build as well
[ -f ../patch_$VERSION ] && echo "Applying patch ../patch_$VERSION" && patch -p1 < ../patch_$VERSION

echo **********************
echo CREATING HOST TOOLS
echo **********************

set -e
./configure && make -j8
#make -j8
set +e
popd >/dev/null

echo **********************
echo CREATING LIBRARY
echo **********************

cmi $1

rm -rf cross_tmp
