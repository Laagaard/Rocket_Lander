@echo off
set dates_list=1-18-2025 1-19-2025 1-25-2025 1-26-2025 ^
2-1-2025 2-2-2025 2-8-2025 2-9-2025 2-15-2025 2-16-2025 2-22-2025 2-23-2025 ^
3-1-2025 3-2-2025 3-8-2025 3-9-2025 3-15-2025 3-16-2025 3-22-2025 3-23-2025 3-29-2025 3-30-2025 ^
4-5-2025 4-6-2025 4-12-2025 4-13-2025 4-19-2025 4-20-2025
set hours_list=07 08 09 10 11 12 13 14 15 16 17 18
set minutes_list=00
(for %%a in (%dates_list%) do (
    (for %%b in (%hours_list%) do (
        (for %%c in (%minutes_list%) do (
            echo %%a-%%b-%%c
            python .\dataset_generation.py %%a %%b %%c
            python .\optimal_trajectory.py %%a %%b %%c
            python .\dnt_trajectories.py %%a %%b %%c
            python .\dnt_generation.py %%a %%b %%c
        ))
    ))
))