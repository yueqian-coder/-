#!/bin/bash
set -e
list="$1"
VARIANTS=$(tr '\n' ',' < "$list" | sed 's/,$//')
VARIANTS="$VARIANTS" /home/user/MRPC-2025-homework/solutions/run_variants.sh