# sokoban_game
2019 U of T CSC384  introduction to AI  assignment 1
## What is this package
This package is the first assignment of CSC384, we are trying to practice different search method( anytime best-frist search and anytime weighted A * search) and compare results from different herustic methods (simple manhattan and a better herustic). This assignment is about a classic game sokoban, abstraction of the games goes as follow:
* State space: each state is represented by a sokuban object,each have the dimension of the space(here we assume the game only in a M*N block for simplicity), set of positions of robots, set of positions of blocks, set of positions of storage(where the block should go) and set of positions of obstacles.
* Initial state: initial state is a sokuban object contain information of the state when the game started
* Desired /Goal condition: all the blocks are on its storage position
* Actions: robot can only push one block(not two concecutive blocks) can not pull, cannot across obstacles.

## What is in this package:
### what we are provided
* search.py: coding for a search engine
* sokoban.py: coding for the sokoban object
* WaterJugs.py: example usage of search.py
* autograder: coding for testing how is our coding
### what we need to write
* solution.py: contains serveral search functions and heuristic distance functions
* tips: tips for writing this assignment

# Quality of my solution
13/20 tests,haven't have the result yet, probably not very good >_<, but it is a fun assignment!

# References
1. our course website(where exist a detailed instruction file): https://www.teach.cs.toronto.edu//~csc384h/winter/lectures.html
2. example solution of the 2017 version with slightly different details by gabrielarpino: https://github.com/gabrielarpino/Sokoban-Solver-AI
