set key autotitle columnhead
set datafile separator ","

plot    'tracking.csv' with lines, \
        'trajectory.csv' with points