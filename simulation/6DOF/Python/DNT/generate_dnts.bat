@echo off
set dates_list=02-23-2025
set hours_list=11 12 13 14 15
(for %%a in (%dates_list%) do (
    (for %%b in (%hours_list%) do (
        python .\dataset_generation.py --date %%a --time %%b --location ROAR --automated False
        python .\optimal_trajectory.py --date %%a --time %%b --location ROAR --automated False
        python .\dnt_trajectories.py --date %%a --time %%b --location ROAR --automated False
        python .\dnt_generation.py --date %%a --time %%b --location ROAR --automated False
    ))
))