#!/bin/bash
set -e
ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
VERSION="824ecdf6dc06ad0560001741ef1db861d4ed069f"

cd $ROOT/tmp

if [ -z ${NUM_JOBS} ]; then
    NUM_JOBS=1
fi

if [ ! -e "${RISCV}/bin/spike"  ]; then
    echo "Installing fesvr"
    git clone https://github.com/riscv/riscv-isa-sim.git
    cd riscv-isa-sim
    git checkout $VERSION
    mkdir -p build
    cd build
    ../configure --prefix="$RISCV/"
    make install-config-hdrs install-hdrs libfesvr.a
    mkdir -p $RISCV/lib
    cp libfesvr.a $RISCV/lib
else
    echo "Using fesvr from cached directory."
fi



