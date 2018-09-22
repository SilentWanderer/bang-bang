set xrange [0:396]
set yrange [0:324]
set xtics 10
set ytics 10

stats 'tracking.csv'
num_records = floor(STATS_records/100.)

plot    'tracking.csv' with lines, \
        'trajectory.csv' with points


plot "field.png" binary filetype=png dx=0.6492 dy=0.6304 w rgbalpha
