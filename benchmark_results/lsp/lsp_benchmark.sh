#!/bin/bash
FOLDER_1="../../motion_planning_datasets/alternating_gaps/train/"
FOLDER_2="../../motion_planning_datasets/bugtrap_forest/train/"
FOLDER_3="../../motion_planning_datasets/forest/train/"
FOLDER_4="../../motion_planning_datasets/gaps_and_forest/train/"
FOLDER_5="../../motion_planning_datasets/mazes/train/"
FOLDER_6="../../motion_planning_datasets/multiple_bugtraps/train/"
FOLDER_7="../../motion_planning_datasets/shifting_gaps/train/"
FOLDER_8="../../motion_planning_datasets/single_bugtrap/train/"


NUM_ENVS=100


cd ../../examples/
python lsp_2d_benchmark.py --database_folders ${FOLDER_1} ${FOLDER_2} ${FOLDER_3} ${FOLDER_4} ${FOLDER_5} ${FOLDER_6} ${FOLDER_7} ${FOLDER_8} --num_envs ${NUM_ENVS} 
