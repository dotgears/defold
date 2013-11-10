#!/bin/sh

readonly BASE_URL=http://googletest.googlecode.com/files
readonly FILE_URL=gtest-1.5.0.tar.gz
readonly PRODUCT=gtest
readonly VERSION=1.5.0

if [ $1 == "darwin" ]; then
    # tr1/tuple isn't available on clang/darwin and gtest 1.5.0 assumes that
    # see corresponding flag in waf_dynamo.py
    export CXXFLAGS='-DGTEST_USE_OWN_TR1_TUPLE=1'
fi

. ../common.sh

download
cmi $1
