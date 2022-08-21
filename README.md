# Graph Algorithms

Implementation of 3 graph algorithms: A*, BFS, and Iterative Deepening. 

# Input File

Input should be a text ﬁle with the following contents.

* Each line in the ﬁle should be either a comment, vertex or an edge.

* A line that starts with # is a comment line and should be skipped A vertex consists of a node label followed by two ints: an x and y coordinate.

* An edge (always undirected) is simply two node labels.

* Note: A node label should be any string with alphanumeric characters 

* Note: there is no guaranteed order. Edges might reference vertices not yet deﬁned, and that is ﬁne as long as they appear somewhere in the ﬁle. 

# How To Build the Program

To build this project use cmake. First create a new directory called build. Go into the build directory and run cmake, then type `make` to build the project. Finally, run the program named `path`.
```
mkdir build
cd build
cmake ..
make
```
# How To Run

path [-v] -start $start-node -goal $goal-node -alg $alg graph-file

* `-v` is an optional ﬂag for verbose mode (more later)

* `-start` is followed by the name of the node to start from

* `-end` is followed by the name of the node which is the goal

* `-alg` is followed by one of: BFS, ID, ASTAR

* `-depth` used only for iterative-deepening, that indicates the initial search depth, with a default increase of 1 after that.

## Examples:
You can run the BFS algorithm using:

```
./path -v --start S --goal G --alg BFS ../data/ex1.txt
```

or you could also type 

```
./path -v -s S -g G -a BFS ../data/ex1.txt
```

The A* and iterative deepening algorithms can be run in the same way using ```ASTAR``` and ```ID``` instead of ```BFS```.

To set the initial depth for iterative deepening, use the ```-d``` or the ```--depth``` options with an integer argument. 

```
./path -v --start S --goal G --alg ID -depth 2 ../data/ex1.txt
```

# Implementation Details

This algorithms were implemented according with the specifications in the AI book. In some ways it was different from the sample output in minor ways but I mentioned this instances in the code. 
