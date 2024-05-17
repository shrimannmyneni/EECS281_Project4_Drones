# EECS281_Project4_Drones
Data Structures and Algorithms Project - Main Topics: Prim's Algorithm, Kruskal's Algorithm, Greedy Algorithms, Branch & Bound Algorithms

# Drone Delivery System

## Project Identifier: 1761414855B69983BD8035097EFBD312EB0527F0

### EECS 281: Project 4 - Drone Delivery

### Overview
This project implements a drone delivery system that calculates optimal paths and minimum spanning trees for a set of delivery locations on a campus. The system is divided into three parts:
1. **Part A: Minimum Spanning Tree (MST)**
2. **Part B: Approximate Travelling Salesperson Problem (TSP)**
3. **Part C: Optimal TSP using Branch and Bound**

### Project Goals
- Implement MST algorithms
- Understand and apply heuristics for TSP
- Use Branch and Bound for optimal solutions
- Develop efficient algorithms for large datasets

### Getting Started

#### Prerequisites
- C++ compiler (GCC 11.3.0 recommended)
- Make

#### Building the Project
1. Clone the repository:
    ```bash
    git clone <repository_url>
    cd <repository_directory>
    ```
2. Compile the project:
    ```bash
    make
    ```

#### Running the Program
1. To run the program in different modes:
    ```bash
    ./drone --mode MST < inputFile.txt
    ./drone --mode FASTTSP < inputFile.txt
    ./drone --mode OPTTSP < inputFile.txt
    ```
2. To display help:
    ```bash
    ./drone -h
    ```

### Input Format
- The input is read from standard input (cin).
- The first line contains an integer `N`, the number of vertices.
- The next `N` lines each contain two integers, representing the x and y coordinates of the vertices.

#### Example Input

### Output Format
- **MST Mode**: Prints the total weight of the MST and the edges in the MST.
- **FASTTSP Mode**: Prints the total length of the tour and the order of nodes visited.
- **OPTTSP Mode**: Prints the optimal tour length and the order of nodes visited.

#### Example Outputs
- **MST Mode**:
    ```
    19.02
    0 1
    2 4
    1 3
    1 4
    ```
- **FASTTSP Mode**:
    ```
    31.64
    0 4 2 3 1
    ```
- **OPTTSP Mode**:
    ```
    31.64
    0 1 3 2 4
    ```

### Implementation Details
- **Part A: MST**:
  - Uses Prim's algorithm to find the Minimum Spanning Tree (MST).
  - Ensures connections only between same area locations or through border locations.

- **Part B: FASTTSP**:
  - Implements a heuristic to find a fast approximation for the Travelling Salesperson Problem (TSP).
  - Starts from an initial path and iteratively improves it.

- **Part C: OPTTSP**:
  - Uses Branch and Bound to find the optimal TSP solution.
  - Prunes branches that cannot yield better solutions than the current best.

### License
This project is proprietary and not intended for redistribution.

### Author
Shrimann Myneni

### Acknowledgements
- University of Michigan EECS 281 course staff for project guidelines and support.
- Online resources for TSP heuristics and MST algorithms.
