set datafile separator ','

set terminal pngcairo enhanced size 1024,768
set output 'kalman_filter_output.png'

set xlabel 'Time (s)'
set ylabel 'Position and Velocity'

# Define line styles
set style line 1 linewidth 1 linecolor rgb "red"  # X_1 (position) measure
set style line 2 linewidth 1 linecolor rgb "blue"  # Y_1 (velocity) measure
set style line 3 linewidth 2 linecolor rgb "green"  # X_1 (position) filtered
set style line 4 linewidth 2 linecolor rgb "orange"  # Y_1 (velocity) filtered

# Plot the data using lines with specified line styles
plot 'kalman_filter_output.txt' using 1:3 with lines linestyle 2 title 'Velocity Noisy Measurements', \
     '' using 1:5 with lines linestyle 4 title 'Velocity Kalman Filtered'

# Automatically saves and closes
