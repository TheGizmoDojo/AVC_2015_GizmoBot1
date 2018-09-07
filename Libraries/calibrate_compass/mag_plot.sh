set datafile separator ","
plot "mag.csv" using 1:2 title "XY" pointsize 2 pointtype 7, "mag.csv" using 2:1 title "YX" pointsize 2 pointtype 7, "mag.csv" using 3:2 title "ZY" pointsize 2 pointtype 7


