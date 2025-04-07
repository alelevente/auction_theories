#!/bin/bash

seeds=(42 1234 1867 613 1001)

for s in ${seeds[@]}; do
    python3 auction_measurement.py $s --name beta10_$s --config ../01_simulation/configurations/beta10.json &
done
