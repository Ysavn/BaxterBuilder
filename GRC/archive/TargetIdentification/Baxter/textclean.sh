#!/bin/bash
./textcleaner -g -e normalize -t 5 -s 2  -u -T cleanedTarget.png output.png
convert output.png -resize 160x80 output2.png
