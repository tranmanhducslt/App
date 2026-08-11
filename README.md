# App (VANET)

Final thesis to figure out a method for self-driving cars connecting to signal transmitters on the road.

* **main.c** stores the main program.
* **benchmark.py** stores the testing program.
* **benchmark_results.csv** stores its results.
* **performance_dashboard.html** formats it visually.

## Main program

To run:
* Install a C compiler if needed (e.g., gcc).
* Compile the main program:
```
gcc main.c -o main
```
* Run the program with attributes as desired:
    * random: Number of vehicles; default = 15.
    * steps: Number of simulation steps; default = 30.
    * lookahead: Number of future steps for each vehicle to plan each step; default = 5.
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
* Information is stored in a .csv file, which is connected to the .html file that can be opened in a supporting web browser.

*Created by Duc Tran in July and August 2026*