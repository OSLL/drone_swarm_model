#!/usr/bin/env python3
import sys
import os
from bagpy import bagreader
import pandas as pd
import rosbag


def extract(bagfile, new_name=""):
    b = bagreader(bagfile)
    topic = list(b.topic_table.Topics)
    if len(topic) == 1:
        csv = b.message_by_topic(topic)
        if new_name:
            new_name = csv.split("/")[0] + f"/{new_name}.csv"
        else:
            new_name = csv.split("/")[0] + f"/{bagfile.replace('.bag', '.csv')}"
        os.rename(csv, new_name)
    elif len(topic) == 2:
        csv1 = b.message_by_topic(topic[0])
        csv2 = b.message_by_topic(topic[1])
        pcsv1 = pd.read_csv(csv1)
        pcsv2 = pd.read_csv(csv2)
        csv = pd.concat([pcsv1, pcsv2], axis="columns")
        if new_name:
            new_name = csv1.split("/")[0] + f"/{new_name}.csv"
        else:
            new_name = csv1.split("/")[0] + f"/{bagfile.replace('.bag', '.csv')}"
        os.remove(csv1)
        os.remove(csv2)
        csv.to_csv(new_name)
    print(f"Successfully! Created file {new_name}")


if __name__ == '__main__':
    try:
        bagfile = sys.argv[1]
        try:
            if sys.argv[2] == '-o':
                new_name = sys.argv[3]
                extract(bagfile, new_name)
                exit()
        except IndexError:
            pass
        extract(bagfile)
    except (rosbag.bag.ROSBagException, IndexError, FileNotFoundError):
        print("Check file and arguments!")
        print("Right command: python3 parse_rosbag.py rosbag_file_name.bag -o name_of_out_file")
        print("-o is optional")
