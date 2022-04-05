from cmath import log
import logging

import seaborn as sns
import pandas as pd
import matplotlib as mlib
import numpy as np
import matplotlib.pyplot as plt

from datetime import datetime
from matplotlib.dates import DateFormatter
from matplotlib.ticker import FuncFormatter

from parameters import *

def plot_frequencies(data, axis):
    
    g = sns.barplot(
        data=data, 
        x="topic", 
        y="hz", 
        hue="Source",
        palette=PALETTE_TUD,
        ax=axis
    )
    g.set(xlabel="Topic", ylabel="Frequency [Hz]")
    
    axis.axes.xaxis.set_visible(False)
    #axis.set_xticklabels(ax.get_xticklabels(), rotation=45, ha='right')

    handles, labels = axis.get_legend_handles_labels()
    axis.legend(handles, labels, 
        title="Aspect"
    )

def plot_proc_time(data, axis):

    # Ensuring the X-Axis always matches across different plots.
    data = data.sort_values(by="topic")
   
    data = pd.melt(
        frame=data[["topic", "processing"]],
        id_vars=["topic"]
    )

    logging.info("Plotting")
    
    g = sns.boxplot(
        data=data, 
        x="topic", 
        y="value", 
        # Would be a bit messy with outliers enabled.
        showfliers = False,
        palette=PALETTE_TUD,
        ax=axis
    )
    g.set_yscale("log")
    g.set(xlabel="Topic", ylabel="Processing Time [µs]")
    
    axis.axes.xaxis.set_visible(False)

def plot_load(data, ax, y_cpu_lim=100, y_mem_scale=0.1):
    
    # Plot only data from the recorder.
    data = data[data["process"] == "record"]
    # Ensuring the X-Axis always matches across different plots.
    data = data.sort_values(by="topic")

    data = pd.melt(
        frame=data[["topic", "cpu", "memory"]],
        id_vars=["topic"]
    )
   
    mask = data.variable.isin(['memory'])
    data.loc[mask, 'value'] = data.loc[mask, 'value'] * y_mem_scale

    g = sns.barplot(
        data=data, 
        x="topic", 
        y="value", 
        hue="variable",
        palette=PALETTE_TUD,
        ax=ax
    )
    g.set(xlabel="", ylabel="CPU Load [%]")
    
    ax.set_ylim([0, y_cpu_lim])
    ax.get_yaxis().set_major_formatter(mlib.ticker.ScalarFormatter())
    ax.legend().remove()

    handles, labels = ax.get_legend_handles_labels()
    ax.legend(handles, map(lambda u: u.capitalize(), labels), 
        title="",
        loc="upper right",
        frameon=True,
        ncol=2
    )

    twin = ax.twinx()
    twin.set_ylim(ax.get_ylim())
    twin.set_yticklabels(np.round(ax.get_yticks() / y_mem_scale, 0).astype(int))
    twin.set_ylabel('Memory Utilization [MB]')
    twin.grid(False)

    mapped = []

    for k in ax.get_xticklabels():
        mapped.append(NAME_MAPPING[k.get_text()])

    # TODO
    ax.set_xticklabels(mapped, rotation=45, ha='right')
    ax.get_xaxis().set_visible(True)


def print_proc_share_simple(data):

    counts = data.groupby(
        by=["topic", "idx", "source"]
    ).count().reset_index()

    logging.info("Printing Processing Share Data.")

    for topic in data["topic"].unique():

        if not topic in NAME_MAPPING:
            continue

        topic_cnt_data = counts[counts["topic"] == topic]
        
        src_cnt = topic_cnt_data[topic_cnt_data["source"] == "actual"].sort_values(by="idx")
        prc_cnt = topic_cnt_data[topic_cnt_data["source"] == "processed"].sort_values(by="idx")
        
        np_processed = prc_cnt["hz"].to_numpy()
        # TODO: Sometimes even source
        np_actual = src_cnt["hz"].to_numpy()
        np_actual[:] = np_actual.max()

        share = np_processed / np_actual

        logging.info(f" + Subject: {NAME_MAPPING[topic]}")
        logging.info(f"   - Avg: " + str(np.mean(share)))
        logging.info(f"   - Std: " + str(np.std(share)))
        logging.info(f"   - Med: " + str(np.median(share)))


def print_frequencies_simple(data):

    data_actual = data[data["source"] == "actual"]

    data_actual_unique = data_actual.groupby(
        by=['topic', 'source', "idx"]
    ).mean().reset_index().drop_duplicates(
        subset=['topic', 'source', "idx"], 
        keep="first"
    )

    logging.info("Printing Frequncy Data.")

    for topic in data["topic"].unique():

        if not topic in NAME_MAPPING:
            continue
        
        data_topic = data_actual_unique[data_actual_unique["topic"] == topic]
        data_hz = data_topic["hz"]
        
        logging.info(f" + Subject: {NAME_MAPPING[topic]}")
        logging.info(f"   - Avg: " + str(data_hz.mean()))
        logging.info(f"   - Std: " + str(data_hz.std()))
        logging.info(f"   - Med: " + str(data_hz.median()))