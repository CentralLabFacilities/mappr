#!/bin/bash

# http://stackoverflow.com/questions/59895/can-a-bash-script-tell-what-directory-its-stored-in
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

PROJECT=mappr_rviz
WORKSPACE=$DIR/../../../build/$PROJECT
FULL="$(cd "$(dirname "$WORKSPACE")"; pwd)/$(basename "$WORKSPACE")"

$DIR/run-clang-tidy.py -p=$FULL -fix
