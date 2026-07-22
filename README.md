Final thesis to figure out a method for self-driving cars connecting to signal transmitters on the road.

* **main.c** stores the main program.
* **benchmark.py** stores the testing program.
* **benchmark_results.csv** stores its results.
* **performance_dashboard.html** formats it visually.

To run:
* Install a C compiler if needed (e.g., gcc).
* Compile the main program:
```
gcc main.c -o main
```
* Run the program with attributes as desired:
    * random: Number of vehicles; default = 15.
    * steps: Number of simulation steps; default = 30.
    * lookahead: Number of steps to plan each step; default = 5.
```
./main
```
```
./main random [#] steps [#] lookahead [#]
```

*Created by Duc Tran in July 2026*