#!/bin/bash

for i in {1..100}
do
    echo "Run $i"
    ./waf --run "scratch/randomAP4user100_prototype"
done
echo "All runs completed."