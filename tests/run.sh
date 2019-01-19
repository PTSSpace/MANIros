#!/bin/bash

set -e # fail on errors
dir=$(dirname "$0")

function log {
  echo ""
  echo "$1"
  echo ""
}

log "[*] Running unittest"
find "$dir" -name 'unit_*.py' -print0 | xargs -0 -n1 python

log "[*] Running integartion tests"
find "$dir" -name '*.test' -print0 | xargs -0 -n1 rostest

