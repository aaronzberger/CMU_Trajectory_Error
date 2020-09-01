READ_FILE = ARG1

set terminal qt 0
set datafile separator ','
set title "Iterations vs. Position Error"
set xlabel "Iteration (Waypt Number)"
set ylabel "Position Error (m from target)"
plot READ_FILE using 1:2 notitle with lines

set terminal qt 1
set datafile separator ','
set title "Iteration vs. Orientation Error"
set xlabel "Iteration (Waypt Number)"
set ylabel "Orientation Error (rad from target)"
plot READ_FILE using 1:3 notitle with lines
pause -1 "Hit any key to continue"