#!/bin/bash

i=0;
while read info; do
    myarray=($info)
    echo ${myarray[0]}_$(printf "%03d_" "$i")
    rosrun camera_controls move.py $info
    rosrun camera_controls save_image.py ${myarray[0]} ${myarray[0]}_$(printf "%03d_" "$i")
    i=$((i + 1))
done < "$1";