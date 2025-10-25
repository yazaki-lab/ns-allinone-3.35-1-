#!/bin/bash

for i in {1..100}
do
    echo "Run $i"
    ./waf --run "scratch/myargoAP4user100_classroom"
done
echo "All runs completed."