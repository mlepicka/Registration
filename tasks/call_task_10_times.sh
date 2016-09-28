#!/bin/bash
for i in {1..10}
do
   eval "discode -T RGBDSequenceSOMCollectorKAZE.xml -L 0"
   wait
   echo "Wykonano wyniki $i -ty raz"
done
