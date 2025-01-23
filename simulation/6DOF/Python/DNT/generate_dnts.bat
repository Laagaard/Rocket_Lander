@echo off
set dates_list=1-25-2025
set hours_list=07 09 11
set minutes_list=00
(for %%a in (%dates_list%) do (
    (for %%b in (%hours_list%) do (
        (for %%c in (%minutes_list%) do (
            python .\dataset_generation.py %%a %%b %%c 1
            python .\optimal_trajectory.py %%a %%b %%c 1
            python .\dnt_trajectories.py %%a %%b %%c 1
            python .\dnt_generation.py %%a %%b %%c 1
        ))
    ))
))