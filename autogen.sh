#!/bin/sh

set -e

cd $(dirname $0)
mkdir -p m4
autoreconf -f -i
./configure "$@"
