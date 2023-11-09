#!/usr/bin/env python
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
path = os.path.expanduser('~/.ros/tracing/simple_callback')

# Process
events = load_file(path)

#######################################################
# THIS IS THE CLASS THAT PROCESSES ALL THIS TRACE FILE
# IT ALLOWS US TO ACCES THAT INFOR WITH METHODS AND CLASSES
handler = Ros2Handler.process(events)
######################################################

data_util = Ros2DataModelUtil(handler.data)

print(data_util)

callback_symbols = data_util.get_callback_symbols()
print("CALLBACK SYMBOLS ================")
print(str(callback_symbols))

# Instead of output_notebook() which is for Jupyter, you will use output_file() for scripts
output_file("ros2_tracing_analysis.html")

# data_util.data.print_data()


######################
# FUNCTIONS


def get_timer_callback_ranges(timer_node_name: str) -> List[Tuple[pd.Timestamp, pd.Timestamp, pd.Timedelta]]:
    # Get timer object
    objs_and_owners = {
        obj: data_util.get_callback_owner_info(obj)
        for obj, _ in callback_symbols.items()
    }


    print("############# objs_and_owners ##############")
    print(objs_and_owners)
    print("############################################")

    timer_objs = [
        obj for obj, owner_info in objs_and_owners.items()
        if timer_node_name in owner_info and 'Timer' in owner_info
    ]
    assert 1 == len(timer_objs), f'len={len(timer_objs)}'
    timer_obj = timer_objs[0]

    # Get callback durations
    callback_durations = data_util.get_callback_durations(timer_obj)

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
        
        numerical_y_position = 0#get_numerical_y_position(segment_type)

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



ranges_timer_robot_status = get_timer_callback_ranges('robot_status')
print(ranges_timer_robot_status)


#########

segment_types = [
    'callback timer robot_status',
]

start_time = "START TIME"
fig = figure(
    title='Robot Status callbacks and publications',
    x_axis_label=f'time (from {start_time})',
    y_range=segment_types,
    width=1200,
    height=600,
)

fig.title.align = 'center'
fig.title.text_font_size = '40px'
fig.xaxis[0].formatter = DatetimeTickFormatter()
fig.xaxis[0].axis_label_text_font_size = '30px'
fig.yaxis[0].major_label_text_font_size = '25px'


add_durations_to_figure(fig, 'callback timer robot_status', ranges_timer_robot_status, 'red')

show(fig)









