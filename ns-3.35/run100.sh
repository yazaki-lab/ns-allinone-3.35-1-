#!/bin/bash

for i in {1..100}
do
    echo "Run $i"
    ./waf --run "scratch/randomAP2user5"
    ./waf --run "scratch/myargoAP2user5_upglade"
done
echo "All runs completed."