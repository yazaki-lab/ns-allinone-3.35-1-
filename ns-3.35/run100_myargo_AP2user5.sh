#!/bin/bash

for i in {1..100}
do
    echo "Run $i"
    ./waf --run "scratch/myargoAP2user5"
done
echo "All runs completed."