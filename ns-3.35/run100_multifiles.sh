#!/bin/bash

FILES=(
  "scratch/myargoAP4user100_entrance.cc"
  "scratch/randomAP4user100_entrance.cc"
  
)

for file in "${FILES[@]}"; do
    echo "Running simulations for $file"
    for i in {1..100}; do
        echo "Run $i for $file"
        ./waf --run "$file"
    done
done

echo "All runs completed."
