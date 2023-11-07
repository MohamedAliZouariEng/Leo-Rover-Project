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

# Process
events = load_file(path)
handler = Ros2Handler.process(events)


# In[6]:


data_util = Ros2DataModelUtil(handler.data)

callback_symbols = data_util.get_callback_symbols()

# Instead of output_notebook() which is for Jupyter, you will use output_file() for scripts
output_file("ros2_tracing_analysis.html")


# In[7]:


data_util.data.print_data()


# In[8]:


# Functions to analyze the pre-processed data

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


def get_pub_sub_creation_time(handle_type: str, topic_name: str) -> pd.Timestamp:
    # Get handle
    handle = get_handle(handle_type, topic_name)
    
    df = None
    if handle_type == 'pub':
        # Get timestamp from rcl_publisher_init
        df = data_util.convert_time_columns(data_util.data.rcl_publishers, [], ['timestamp'], False)
    elif handle_type == 'sub':
        # Get timestamp from rcl_subscription_init
        df = data_util.convert_time_columns(data_util.data.rcl_subscriptions, [], ['timestamp'], False)
    else:
        assert False, 'unknown handle_type value'
    return df.loc[handle, 'timestamp']


def get_timer_creation_time(node_name: str) -> pd.Timestamp:
    # Get handle
    node_handle = get_handle('node', node_name)
    
    # Get timer handle from timer-node links
    timer_node_links = data_util.data.timer_node_links.loc[
        data_util.data.timer_node_links['node_handle'] == node_handle
    ].index.values.astype(int)
    assert 1 == len(timer_node_links), f'len={len(timer_node_links)}'
    timer_handle = timer_node_links[0]
    
    # Get creation timestamp
    df = data_util.convert_time_columns(data_util.data.timers, [], ['timestamp'], False)
    return df.loc[timer_handle, 'timestamp']


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


def get_sub_callback_ranges(sub_topic_name: str) -> List[Tuple[pd.Timestamp, pd.Timestamp, pd.Timedelta]]:
    # Get callback object
    objs_and_owners = {
        obj: data_util.get_callback_owner_info(obj)
        for obj, _ in callback_symbols.items()
    }
    sub_objs = [
        obj for obj, owner_info in objs_and_owners.items()
        if sub_topic_name in owner_info
    ]
    assert 1 == len(sub_objs), f'len={len(sub_objs)}'
    sub_obj = sub_objs[0]

    # Get callback durations
    callback_durations = data_util.get_callback_durations(sub_obj)

    # Convert to simple list of tuples
    ranges = []
    for _, row in callback_durations.iterrows():
        begin = row['timestamp']
        duration = row['duration']
        ranges.append((begin, begin + duration, duration))
    return ranges


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


# In[9]:


# Functions to display data

def add_durations_to_figure(
    fig,  # Changed 'figure' to 'fig' to match the passed argument
    segment_type: str,
    durations: List[Union[Tuple[dt.datetime, dt.datetime]]],
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


# In[10]:


# Analyze and display

creation_timer_ping = get_timer_creation_time('ping')
creation_pub_ping = get_pub_sub_creation_time('pub', '/ping')
creation_pub_pong = get_pub_sub_creation_time('pub', '/pong')
creation_sub_ping = get_pub_sub_creation_time('sub', '/ping')
creation_sub_pong = get_pub_sub_creation_time('sub', '/pong')
ranges_timer_ping = get_timer_callback_ranges('ping')
ranges_sub_ping = get_sub_callback_ranges('/ping')
ranges_sub_pong = get_sub_callback_ranges('/pong')
times_pub_ping = get_publish_times('/ping')
times_pub_pong = get_publish_times('/pong')

# For some reason it seems to be displayed in the reverse order on the Y axis
segment_types = [
    'publish /pong',
    'callback /ping',
    'publish /ping',
    'callback /pong',
    'callback timer ping',
]
start_time = ranges_timer_ping[0][0].strftime('%Y-%m-%d %H:%M')
fig = figure(
    title='Ping-pong callbacks and publications',
    x_axis_label=f'time (from {start_time})',
    y_range=segment_types,
    plot_width=2000,
    plot_height=600,
)
fig.title.align = 'center'
fig.title.text_font_size = '40px'
fig.xaxis[0].formatter = DatetimeTickFormatter()
fig.xaxis[0].axis_label_text_font_size = '30px'
fig.yaxis[0].major_label_text_font_size = '25px'

add_markers_to_figure(fig, 'callback timer ping', [creation_timer_ping], 'blue', marker_type='plus')
add_markers_to_figure(fig, 'publish /ping', [creation_pub_ping], 'blue', marker_type='plus')
add_markers_to_figure(fig, 'publish /pong', [creation_pub_pong], 'red', marker_type='plus')
add_markers_to_figure(fig, 'callback /ping', [creation_sub_ping], 'red', marker_type='plus')
add_markers_to_figure(fig, 'callback /pong', [creation_sub_pong], 'blue', marker_type='plus')
add_durations_to_figure(fig, 'callback timer ping', ranges_timer_ping, 'blue')
add_durations_to_figure(fig, 'callback /pong', ranges_sub_pong, 'blue')
add_markers_to_figure(fig, 'publish /ping', times_pub_ping, 'blue', legend_label='ping node')
add_durations_to_figure(fig, 'callback /ping', ranges_sub_ping, 'red')
add_markers_to_figure(fig, 'publish /pong', times_pub_pong, 'red', legend_label='pong node')

show(fig)


# In[ ]:



