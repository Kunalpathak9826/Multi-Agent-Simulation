set view map
set xrange [0:*]
set yrange [0:*]
set cbrange [0:3]  # Adjust based on max value in your data
set palette defined (0 "white", 1 "black", 2 "blue", 3 "red")

# Plot both map and task data
plot 'kiva-10-500-5_map.txt' matrix with image title "Map Data"
replot 'kiva-0.2_task.txt' matrix with image title "Task Data"
pause -1  # Keeps the plot open
