#!/bin/python3
import os
from pathlib import Path
import pandas as pd

from rosbags.dataframe import get_dataframe
from rosbags.highlevel import AnyReader
from rosbags.typesys import get_types_from_msg, register_types


workspace_directory='/home/hugobaptista/ros2/AIML'


# Add all topic message types to the reader's registered types
add_types = {}
for filename in os.listdir(f'{workspace_directory}/src/ccpm_msgs/msg'):
    text = Path(f'{workspace_directory}/src/ccpm_msgs/msg/{filename}').read_text()
    name=f'ccpm_msgs/msg/{filename.split(".")[0]}'
    add_types.update(get_types_from_msg(text, name))
register_types(add_types)


# Read and convert the topics from the bag file to pandas dataframe
with AnyReader([Path(f'{workspace_directory}/AIML_bag_191223')]) as reader:
    # The 'topics' dictionary's keys are the topics names and the values are their arguments (list)
    topics={}
    for connection in reader.connections:
        keys = []
        for field in reader.typestore.FIELDDEFS[connection.msgtype][1]:
            keys.append(field[0])
        topics[connection.topic]=keys
    
    # The topics /motor0/status and /motor1/status are removed because they break the get_dataframe function
    topics.pop('/motor0/status')
    topics.pop('/motor1/status')

    # The 'dataframes' dictionary's keys are the topics names and the values are their pandas dataframe
    dataframes={}
    for topic in topics:
        dataframes[topic]=get_dataframe(reader, topic, topics.get(topic))

    for topic in dataframes:
        print(f'Topic: {topic}\nDataframe:{dataframes[topic].head}\n-----------------------------------------\n')