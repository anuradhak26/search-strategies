# Search Strategies
Implement and compare different search strategies. This project compares different uninformed and informed search strategies.  

Uninformed search strategies | bfs, dfs, iddfs, unicost 
---|---
Informed search strategies   | greedy, astar, idastar

These search alogorithms are compared by solving these 3 puzzles i.e. the water jug, path-planning and pancakes. 

## Run

The python code accepts three arguments :  
1. configuration file for the puzzle
2. search algorithm  
3. different heuristic function for informed search algorithms (optional)  

Here are examples to run the program :  
`python src/search_strategies.py code/test_cities.config dfs`  
`python src/search_strategies.py code/test_cities.config astar manhattan`
