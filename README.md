# App (VANET)

Final thesis to figure out a method for self-driving cars connecting to signal transmitters on the road.

* **main.c** stores the main program.
* **benchmark.py** stores the testing program.
* **benchmark_results.csv** and **benchmark_vehicles_results.csv** store its results.
* **performance_dashboard.html** formats them visually.
* **routes.txt** saves the overall route of each vehicle in the latest run.

## Main program

To run:
* Install a C compiler if needed (e.g., gcc or clang).
* Compile the main program:
```
gcc main.c -o main
```
* Run the program with attributes as desired:
    * random: Number of vehicles; default = 15.
    * steps: Number of simulation steps; default = 30.
    * lookahead: Number of future steps for planning in each turn; default = 5.
```
./main
```
```
./main random [#] steps [#] lookahead [#]
```

## Benchmark file

The benchmark program tests different values of lookahead and returns the result in a .html file. 

To run:
* Install Python. (May come up as python3)
* Run the benchmark:
```
python3 benchmark.py
```
* Information is stored in a .csv file, which is connected to the .html file that can be opened in a supporting web browser (e.g., Firefox or Chrome).

*Created by Duc Tran in July and August 2026*
