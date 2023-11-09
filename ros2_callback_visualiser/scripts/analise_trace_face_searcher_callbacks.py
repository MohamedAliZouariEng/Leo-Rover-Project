#!/usr/bin/env python3
# coding: utf-8

# The above line is the shebang which tells the system this is a Python script

import os
import sys
import datetime as dt
from typing import List, Tuple, Union, Optional

from bokeh.plotting import figure, output_file, show
from bokeh.layouts import row
from bokeh.models import ColumnDataSource, DatetimeTickFormatter, Segment
import numpy as np
import pandas as pd

# Assuming the `tracetools_analysis` and `tracetools_read` modules are in the PYTHONPATH
from tracetools_analysis.loading import load_file
from tracetools_analysis.processor.ros2 import Ros2Handler
from tracetools_analysis.utils.ros2 import Ros2DataModelUtil

# Here you set your data path
path = os.path.expanduser('~/.ros/tracing/face_searcher')

# Process
events = load_file(path)

#######################################################
# THIS IS THE CLASS THAT PROCESSES ALL THIS TRACE FILE
# IT ALLOWS US TO ACCES THAT INFOR WITH METHODS AND CLASSES
handler = Ros2Handler.process(events)
######################################################

data_util = Ros2DataModelUtil(handler.data)

callback_symbols = data_util.get_callback_symbols()
print("CALLBACK SYMBOLS ================")
print(str(callback_symbols))

# Instead of output_notebook() which is for Jupyter, you will use output_file() for scripts
output_file("ros2_tracing_analysis.html")


######################
# FUNCTIONS

def get_sub_callback_ranges(sub_callback_name: str, sub_topic_name: str, sub_node_name: str) -> List[Tuple[pd.Timestamp, pd.Timestamp, pd.Timedelta]]:
    # Get callback object, iterating over the callbacksymbols keys dictionary
    # We get a dictionary with {CallbackSymbols: CallbackOwnerInfo}
    objs_and_owners = {
        obj: data_util.get_callback_owner_info(obj)
        for obj, callback_info in callback_symbols.items()
        if sub_callback_name in callback_info
    }


    # We search for the callback info that has that topic name
    # We are looking for a Subcriber callback to the topic 'sub_topic_name'
    # This will have a structure like this:
    # 94800182533976: 'Subscription -- node: X, tid: 72702, topic: sub_topic_name'
    subscriber_callback_topic_string_parser = "topic: "+sub_topic_name
    susbcription_node_string_parser = "Subscription -- node: "+sub_node_name
    # The callback owner info has to state that
    # 1) Its a Subscrition node
    # 2) Its a topic with the name sub_topic_name
    sub_objs = [
        obj for obj, owner_info in objs_and_owners.items()
        if subscriber_callback_topic_string_parser in owner_info and susbcription_node_string_parser in owner_info
    ]

    # We check that only ONE exists, because it make sno sense to have two subscriber to sam etopic in the same node
    assert 1 == len(sub_objs), "In a node there should be only ONE subscriber to the same topic, "+f'len={len(sub_objs)}'
    sub_obj = sub_objs[0]

    # Get callback durations
    # In practical terms, a DataFrame can be thought of as a dict-like container for Series objects, 
    # and it's commonly used for representing and working with data in a tabular form, 
    # where you have rows and columns, similar to a spreadsheet or a SQL table.
    callback_durations = data_util.get_callback_durations(sub_obj)


    # Convert to simple list of tuples
    ranges = []
    for _, row in callback_durations.iterrows():
        begin = row['timestamp']
        duration = row['duration']
        ranges.append((begin, begin + duration, duration))
    return ranges



def get_service_callback_ranges(sub_callback_name: str) -> List[Tuple[pd.Timestamp, pd.Timestamp, pd.Timedelta]]:
    # Get callback object, iterating over the callbacksymbols keys dictionary
    # We get a dictionary with {CallbackSymbols: CallbackOwnerInfo}
    sub_objs = [
        obj
        for obj, callback_info in callback_symbols.items()
        if sub_callback_name in callback_info
    ]

   
    # We check that only ONE exists, because it make sno sense to have two subscriber to sam etopic in the same node
    assert 1 == len(sub_objs), "In a node there should be only ONE subscriber to the same topic, "+f'len={len(sub_objs)}'
    sub_obj = sub_objs[0]

    # Get callback durations
    # In practical terms, a DataFrame can be thought of as a dict-like container for Series objects, 
    # and it's commonly used for representing and working with data in a tabular form, 
    # where you have rows and columns, similar to a spreadsheet or a SQL table.
    callback_durations = data_util.get_callback_durations(sub_obj)

    # Convert to simple list of tuples
    ranges = []
    for _, row in callback_durations.iterrows():
        begin = row['timestamp']
        duration = row['duration']
        ranges.append((begin, begin + duration, duration))
    return ranges
######################

######################
# DRAWING FUNCTIONS
def add_durations_to_figure(
    fig,  # Changed 'figure' to 'fig' to match the passed argument
    segment_type: str,
    durations: List[Tuple[pd.Timestamp, pd.Timestamp, pd.Timedelta]],
    color: str,
    line_width: int = 60,
    numerical_pos: int = 0,
    legend_label: Optional[str] = None,    
) -> None:
    for duration in durations:
        duration_begin, duration_end, _ = duration

        base_kwargs = dict()
        if legend_label:
            base_kwargs['legend_label'] = legend_label
            
        # Draw the line for the duration
        fig.line(
            x=[duration_begin, duration_end],
            y=[segment_type, segment_type],
            color=color,
            line_width=line_width,
            **base_kwargs,
        )
        
        numerical_y_position = numerical_pos

        y_offset = 0.6  # Adjust this value as needed to raise the text above the line
        text_y_position = numerical_y_position + y_offset

        fig.text(
            x=[duration_begin], y=[text_y_position],
            text=["START"],  # Convert to string if not already
            text_font_size='12pt',  # Adjust font size as needed
            text_baseline="bottom",  # Adjust this as needed
            text_align="left",
            color=color
        )

        y_offset = 0.4  # Adjust this value as needed to raise the text above the line
        text_y_position = numerical_y_position + y_offset

        fig.text(
            x=[duration_end], y=[text_y_position],
            text=["END"],  # Convert to string if not already
            text_font_size='12pt',  # Adjust font size as needed
            text_baseline="bottom",  # Adjust this as needed
            text_align="left",
            color=color
        )
########################################

ranges_listener_callback = get_sub_callback_ranges('listener_callback', '/deepmind_robot1/deepmind_robot1_camera/image_raw', 'face_detector_node')
ranges_detect_faces_callback = get_service_callback_ranges('detect_faces_callback')


#########

segment_types = [
    'listener_callback',
    'detect_faces_callback',
]

start_time = "START TIME"
fig = figure(
    title='Face Searcher Callbacks timeline',
    x_axis_label=f'time (from {start_time})',
    y_range=segment_types,
    width=1800,
    height=900,
)

fig.title.align = 'center'
fig.title.text_font_size = '40px'
fig.xaxis[0].formatter = DatetimeTickFormatter()
fig.xaxis[0].axis_label_text_font_size = '30px'
fig.yaxis[0].major_label_text_font_size = '25px'


line_width = 60

search_string = 'listener_callback'
num_pos_listener_callback = None
for index, item in enumerate(segment_types):
    if search_string in item:
        num_pos_listener_callback = index
        break
else:
    print(f"'{search_string}' not found in the list")


search_string = 'detect_faces_callback'
num_pos_detect_faces_callback = None
for index, item in enumerate(segment_types):
    if search_string in item:
        num_pos_detect_faces_callback = index
        break
else:
    print(f"'{search_string}' not found in the list")

add_durations_to_figure(fig, 'listener_callback', ranges_listener_callback, 'red', line_width, num_pos_listener_callback)
add_durations_to_figure(fig, 'detect_faces_callback', ranges_detect_faces_callback, 'blue', line_width, num_pos_detect_faces_callback)

show(fig)