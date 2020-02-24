#!/bin/bash

cd $(git rev-parse --show-toplevel)/build

common_args="--matcher-type=MAT_BF --selector-type=SEL_KNN --visualize-objects=false --visualize-3d-objects=false --visualize-final-results=false"

for detector in SHITOMASI HARRIS FAST BRISK ORB AKAZE SIFT; do
  for descriptor in BRISK BRIEF ORB FREAK SIFT; do
    cmd="./3D_object_tracking --detector-type $detector --descriptor-type $descriptor $common_args"
    echo === $cmd ===
    $cmd
  done
done

cmd="./3D_object_tracking --detector-type AKAZE --descriptor-type AKAZE $common_args"
echo === $cmd ===
$cmd
