#!/bin/bash

mode="ite"
mkdir -p tfl_samples
DIR_NAME=""
for file in $(cat file.txt); do
    cp -r ${DIR_NAME}/${file} tfl_samples
done