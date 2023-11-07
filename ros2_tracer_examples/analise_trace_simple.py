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
path = os.path.expanduser('~/.ros/tracing/ros-world-2021-demo')
#path = os.path.expanduser('~/.ros/tracing/my-tracing-session/')

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

# data_util.data.print_data()


######################
# FUNCTIONS

def get_sub_callback_ranges(sub_topic_name: str, sub_node_name: str) -> List[Tuple[pd.Timestamp, pd.Timestamp, pd.Timedelta]]:
    # Get callback object, iterating over the callbacksymbols keys dictionary
    # We get a dictionary with {CallbackSymbols: CallbackOwnerInfo}
    objs_and_owners = {
        obj: data_util.get_callback_owner_info(obj)
        for obj, _ in callback_symbols.items()
    }

    print("####################")
    print(str(objs_and_owners))
    print("####################")

    # We search for the callback info that has that topic name
    # We are looking for a Subcriber callback to the topic 'sub_topic_name'
    # This will have a structure like this:
    # 94800182533976: 'Subscription -- node: X, tid: 72702, topic: sub_topic_name'
    subscriber_callback_topic_string_parser = "topic: "+sub_topic_name
    susbcription_node_string_parser = "Subscription -- node: "+sub_node_name
    # The callback onwer info has to state that
    # 1) Its a Subscrition node
    # 2) Its a topic with the name sub_topic_name
    sub_objs = [
        obj for obj, owner_info in objs_and_owners.items()
        if subscriber_callback_topic_string_parser in owner_info and susbcription_node_string_parser in owner_info
    ]

    print("######## Callback for subscriber topic "+sub_topic_name+", Inside Node:"+sub_node_name+"############")
    print(str(sub_objs))
    print("####################")

    # We check that only ONE exists, because it make sno sense to have two subscriber to sam etopic in the same node
    assert 1 == len(sub_objs), "In a node there should be only ONE subscriber to the same topic, "+f'len={len(sub_objs)}'
    sub_obj = sub_objs[0]

    # Get callback durations
    # In practical terms, a DataFrame can be thought of as a dict-like container for Series objects, 
    # and it's commonly used for representing and working with data in a tabular form, 
    # where you have rows and columns, similar to a spreadsheet or a SQL table.
    callback_durations = data_util.get_callback_durations(sub_obj)

    print("######### callback_durations ###########")
    print(str(callback_durations))
    print("####################")


    # Convert to simple list of tuples
    ranges = []
    for _, row in callback_durations.iterrows():
        begin = row['timestamp']
        duration = row['duration']
        ranges.append((begin, begin + duration, duration))
    return ranges


def get_handle(handle_type: str, name: str) -> int:
    if handle_type == 'pub':
        pub_handles = data_util.data.rcl_publishers.loc[
            data_util.data.rcl_publishers['topic_name'] == name
        ].index.values.astype(int)
        # For this demo, we don't expect more than 1 publisher per topic
        assert 1 == len(pub_handles)
        return pub_handles[0]
    if handle_type == 'sub':
        sub_handles = data_util.data.rcl_subscriptions.loc[
            data_util.data.rcl_subscriptions['topic_name'] == name
        ].index.values.astype(int)
        # For this demo, we don't expect more than 1 subscription per topic
        assert 1 == len(sub_handles), f'len={len(sub_handles)}'
        return sub_handles[0]
    if handle_type == 'node':
        node_handles = data_util.data.nodes.loc[
            data_util.data.nodes['name'] == name
        ].index.values.astype(int)
        assert 1 == len(node_handles), f'len={len(node_handles)}'
        return node_handles[0]
    assert False, 'unknown handle_type value'

def get_publish_times(pub_topic_name: str) -> List[pd.Timestamp]:
    # Get all publish instances
    pub_instances = data_util.get_publish_instances()

    # Get publisher handle
    pub_handle = get_handle('pub', pub_topic_name)

    # Since publish calls go rclcpp->rcl->rmw and since we
    # only know the publisher handle at the rcl level, we first
    # get the indexes of all rcl_publish calls for our publisher
    rcl_pub_indexes = pub_instances.loc[
        pub_instances['publisher_handle'] == pub_handle
    ].index.values.astype(int)
    rcl_pub_indexes = list(rcl_pub_indexes)
    max_index = pub_instances.index.values.astype(int).max()
    # Then we group rclcpp & rmw calls (before & after, respectively) with matching message pointers
    rclcpp_rmw_pub_timestamps = []
    for rcl_pub_index in rcl_pub_indexes:
        # Get message pointer value
        message = pub_instances.iloc[rcl_pub_index]['message']
        # Get corresponding rclcpp_publish row
        rclcpp_timestamp = None
        rclcpp_pub_index = rcl_pub_index - 1
        while rclcpp_pub_index >= 0:
            # Find the first row above with the same message
            row = pub_instances.iloc[rclcpp_pub_index]
            if message == row['message'] and 'rclcpp' == row['layer']:
                rclcpp_timestamp = row['timestamp']
                break
            rclcpp_pub_index -= 1
        # Get corresponding rmw_publish row
        rmw_timestamp = None
        rmw_pub_index = rcl_pub_index + 1
        while rmw_pub_index <= max_index:
            # Find the first row below rcl_publish row with the same message
            row = pub_instances.iloc[rmw_pub_index]
            if message == row['message'] and 'rmw' == row['layer']:
                rmw_timestamp = row['timestamp']
                break
            rmw_pub_index += 1

        assert rclcpp_timestamp is not None and rmw_timestamp is not None
        rclcpp_rmw_pub_timestamps.append(rclcpp_timestamp + (rmw_timestamp - rclcpp_timestamp) / 2)

    return rclcpp_rmw_pub_timestamps

def get_timer_callback_ranges(timer_node_name: str) -> List[Tuple[pd.Timestamp, pd.Timestamp, pd.Timedelta]]:
    # Get timer object
    objs_and_owners = {
        obj: data_util.get_callback_owner_info(obj)
        for obj, _ in callback_symbols.items()
    }
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
        fig.line(
            x=[duration_begin, duration_end],
            y=[segment_type, segment_type],
            color=color,
            line_width=line_width,
            **base_kwargs,
        )


def add_markers_to_figure(
    fig,
    segment_type: str,
    times: List[dt.datetime],
    color: str,
    line_width: int = 60,
    legend_label: Optional[str] = None,
    size: int = 30,
    marker_type: str = 'diamond',
) -> None:
    for time in times:
        base_kwargs = dict()
        if legend_label:
            base_kwargs['legend_label'] = legend_label
        if marker_type == 'diamond':
            fig.diamond(
                x=[time],
                y=[segment_type],
                fill_color=color,
                line_color=color,
                size=size,
                **base_kwargs,
            )
        elif marker_type == 'plus':
            fig.plus(
                x=[time],
                y=[segment_type],
                fill_color=color,
                line_color=color,
                size=size,
                **base_kwargs,
            )
        else:
            assert False, 'invalid marker_type value'

######################


ranges_sub_ping = get_sub_callback_ranges('/ping', 'pong')
ranges_sub_pong = get_sub_callback_ranges('/pong', 'ping')

times_pub_ping = get_publish_times('/ping')
times_pub_pong = get_publish_times('/pong')

ranges_timer_ping = get_timer_callback_ranges('ping')


# For some reason it seems to be displayed in the reverse order on the Y axis
segment_types = [
    'publish /pong',
    'callback /ping',
    'publish /ping',
    'callback /pong',
    'callback timer ping',
]
#start_time = ranges_timer_ping[0][0].strftime('%Y-%m-%d %H:%M')
start_time = "START TIME"
fig = figure(
    title='Ping-pong callbacks and publications',
    x_axis_label=f'time (from {start_time})',
    y_range=segment_types,
    width=2000,
    height=600,
)
fig.title.align = 'center'
fig.title.text_font_size = '40px'
fig.xaxis[0].formatter = DatetimeTickFormatter()
fig.xaxis[0].axis_label_text_font_size = '30px'
fig.yaxis[0].major_label_text_font_size = '25px'

add_durations_to_figure(fig, 'callback /ping', ranges_sub_ping, 'red')
add_durations_to_figure(fig, 'callback /pong', ranges_sub_pong, 'blue')

add_markers_to_figure(fig, 'publish /ping', times_pub_ping, 'red', legend_label='ping node')
add_markers_to_figure(fig, 'publish /pong', times_pub_pong, 'blue', legend_label='pong node')

add_durations_to_figure(fig, 'callback timer ping', ranges_timer_ping, 'black')

show(fig)









