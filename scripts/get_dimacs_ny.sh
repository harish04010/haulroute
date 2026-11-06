#!/usr/bin/env bash
# Downloads the New York City road network (travel times) from the
# 9th DIMACS Implementation Challenge: about 264k nodes and 734k arcs.
set -euo pipefail
mkdir -p data && cd data
base="http://www.diag.uniroma1.it/challenge9/data/USA-road-t"
for f in USA-road-t.NY.gr.gz USA-road-t.NY.co.gz; do
  [ -f "${f%.gz}" ] || { curl -fL -O "$base/$f" && gunzip -f "$f"; }
done
echo "Run: ./build/haulroute bench --gr data/USA-road-t.NY.gr --co data/USA-road-t.NY.co"
