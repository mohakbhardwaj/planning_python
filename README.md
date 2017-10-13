# Motion Planning in Python
**********

An easy-to-use yet efficient motion planning library with focus on providing a modular framework for integrating planning algorithms with machine learning frameworks. The library allows for loading planning environments from png files and currently supports lattice based planning with custom heuristics and cost functions.  


## Planning Pipeline Overview
The search based planning pipeline looks as follows:



## External Dependencies
1. [Numpy](http://www.numpy.org/)
2. [Matplotlib](https://matplotlib.org/)
3. [dubins](https://pypi.python.org/pypi/dubins/)


## Installation
Once you have the external dependencies installed, 
  
## Examples

- [minimal_astar_example.py](examples/minimal_astar_example.py):
  A simple example that loads an environment and runs a 2D astar planner with heuristic weight 1. The details of the parameters used are as follows:


- [astar_2d_benchmark_example.py](examples/astar_2d_benchmark_example.py):
  This script takes as input the relative location of different planning problem databases and number of environments in each database. It then runs a 2d astar planner with changing heuristic weights and stores the results in the [benchmark_results](benchmark_results/) folder
  

## API Reference

## Benchmark Results
 We provide results of benchmarks run on different motion planning datasets. We run A-star with varying the weight on the heuristic function from 0 to 91. A heuristic weight of 0 makes the algorithm equivalent to Dijkstra's Algorithm whereas a higher weight makes the algorithm more greedy. The results reported are *average number of expansions* on a set of 100 environments for each database. 

 - **A-star 2D Benchmark Results**
   ```
   Parameters:  graph size                    = 200x200
                connectivity                  = eight-connected
                cost                          = path length
                heuristic                     = Euclidean distance to goal
                collision checking resolution = 1m
    ```

| Heuristic Weight (w) 	| Alternating Gaps 	| Single Bugtrap 	| Shifting Gap 	| Forest 	| Bugtrap+Forest 	| Gaps+Forest 	| Mazes 	| Multiple Bugtrap 	|
|:--------------------:	|:----------------:	|:--------------:	|:------------:	|:------:	|:--------------:	|:-----------:	|:-----:	|:----------------:	|
|           0          	|       31479      	|      37947     	|     31319    	|  33742 	|      33964     	|    25278    	| 18079 	|       36765      	|
|           1          	|       17329      	|      13962     	|     13299    	|  12848 	|      18730     	|    20272    	| 12152 	|       20253      	|
|          11          	|       5393       	|      1138      	|     2479     	|   308  	|      2086      	|     9645    	|  1025 	|       3664       	|
|          21          	|       5684       	|      1126      	|     2615     	|   305  	|      2014      	|     8895    	|  963  	|       3599       	|
|          31          	|       5786       	|      1121      	|     2662     	|   304  	|      1903      	|     8666    	|  942  	|       3578       	|
|          41          	|       5837       	|      1120      	|     2686     	|   301  	|      1890      	|     8552    	|  931  	|       3568       	|
|          51          	|       5868       	|      1119      	|     2700     	|   301  	|      1882      	|     8487    	|  925  	|       3562       	|
|          61          	|       5888       	|      1118      	|     2709     	|   300  	|      1878      	|     8445    	|  921  	|       3558       	|
|          71          	|       5902       	|      1117      	|     2717     	|   300  	|      1875      	|     8414    	|  918  	|       3555       	|
|          81          	|       5913       	|      1117      	|     2722     	|   300  	|      1873      	|     8391    	|  915  	|       3553       	|
|          91          	|       5921       	|      1117      	|     2725     	|   300  	|      1871      	|     8372    	|  914  	|       3551       	|

## Tests

## Contributors

## License