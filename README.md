# SOP
Parallel Solver for the Sequential Ordering Problem

To compile the program run "make clean" and then "make". For an optimizated version compile with "make release".
The program is written for a linux system with c++14 

To run the program on a single instance run "./main <instance_file_path> <time limit> <hash table size> [mode] [per_node] <number of thread>".

instance file path: the path to the intsance file. Assumes a white space separated cost matrix representing the SOP.
Negative values indicate precedences where the row must precede the column.

time limit: the time limit for the program to run (by default the total time limit)

hash table size: the number of buckets for the separate chaining hash table. This is fixed throughout the life of the program.
The number of elements in a bucket is dynamic.

mode (optional): use "full" for verbose printing. Otherwise, prints comma separated values for the program output.

per node: use "per_node" to change the total program time limit to be the instance size multipled by the <time limit> parameter.
Othwerise the <time limit> is the total time limit.

number of threads: the number of threads to run in parallel to solve the problem.

Example usage:

./main tsplib/ESC47.sop 20 1237935 4

verbose printing

./main tsplib/ESC47.sop 20 1237935 full 4

time limit set per node

./main tsplib/ESC47.sop 20 1237935 full per_node 4


To run the sequential version switch to the sequential branch and omit the last parameter.


Example sequential:

./main tsplib/ESC47.sop 20 1237935

Three batch files are included for running several programs in succession (one for each benchmark suite).
In each batch file update the  program parameters for suitable testing purposes.
