# Motion Planning 

## Overview
In this assignment, we implement some Search Based and Sampling Based motion planning algorithms to enable a robot catch a moving target. 

There are totally 11 scenes we test our motion planning algorithm. The 11 tests are implemented in **main.py** as functions `test_map0()`, `test_map1()`, ..., `test_map3c()`.

We implement variations of the A* algorithm such as Agent Centered Search and Anytime Search for the search based motion planning, and Rapidly Exploring Random Tree(RRT) algorithm for the sampling based motion planning algorithm. 

## Environment Setup

1. Create a conda environment using the ece276b.yaml file provided. 
<pre>conda env create -f ece276b.yaml </pre>
2. Activate the conda environment. 
<pre>conda activate ece276b </pre>

### File description

#### 1. robotplanner.py
This script contains all the Search Based motion planning algorithms implemented as python classes. We implement three classes : RTAA that implements the agent centered search, AnytimeA_star to implement the anytime search and A_star that implements a normal epsilon sub-optimal A* algorithm specifically for use in Map7 which is a challenging map because of its large size and complexity. To implement any algorithm, a planner of a particular class is created and then the plan() function of the object is called to get the next robot move. 

#### 2. targetplanner.py

This file contains the target planner. It uses the [minimax decisiton rule](https://en.wikipedia.org/wiki/Minimax) and tries to maximize the minimal distance the robot can achieve. You should not modify this file.

#### 3. main.py

This file contains test functions. It creates the planner as a particular  class of the Motion Planning algorithms and calls the plan() function of the planner object to get the next robot move. The file is run along with three possible command line arguments namely : ara to create an Anytime Search based planner, rtaa to create an Agent Centered Search based planner or rrt to create a Sampling Based RRT planner. After each robot move, the targetplanner function is called to get the next move of the target according to the MiniMax decision. The whole process is then repeated in a for loop until the robot catches up with the target. 
Usage : 
<pre>python3 main.py rtaa #Runs the agent centered planner</pre>
<pre>python3 main.,py ara #Runs the anytime search planner</pre>
<pre>python3 main.py rrt #Runs the sampling based rrt planner</pre>

To run the motion planning algorithm on different test maps, change line 273 test_map portion to one of the test cases defined above. For ex :
<pre>caught, numofmoves = test_map1b()</pre>
<pre>caught, numofmoves = test_map5()</pre>


## Directory
<pre>
starter_code
├── gif
│   ├── ara
│   └── rtaa
├── images
├── maps
├── __pycache__
└── src
    ├── rrt
    │   └── __pycache__
    ├── search_space
    │   └── __pycache__
    └── utilities
        └── __pycache__
└── main.py
└── robotplanner.py
└── targetplanner.py
└── modify_map.py
</pre>
Results of the motion planning algorithm for the different test cases are shown in the gif folder and the images folder. 

## Results
Below we show the results for the different test maps using the agent centered and anytime search algorithms. 

| map0 |
<p float='left'>
<img src="gif/ara/map0.gif" width="49%" title="Anytime Search"> 
<img src="gif/rtaa/map0.gif" width="49%" title="Agent Centered Search">
</p>

| map1 |
<p float='left'>
<img src="gif/ara/map1.gif" width="49%" title="Anytime Search"> 
<img src="gif/rtaa/map1.gif" width="49%" title="Agent Centered Search">
</p>

| map1b |
<p float='left'>
<img src="gif/ara/map1b.gif" width="49%" title="Anytime Search"> 
<img src="gif/rtaa/map1b.gif" width="49%" title="Agent Centered Search">
</p>
| map2 |
<p float='left'>
<img src="gif/ara/map2.gif" width="49%" title="Anytime Search"> 
<img src="gif/rtaa/map2.gif" width="49%" title="Agent Centered Search">
</p>
| map3 |
<p float='left'>
<img src="gif/ara/map3.gif" width="49%" title="Anytime Search"> 
<img src="gif/rtaa/map3.gif" width="49%" title="Agent Centered Search">
</p>
| map3b |
<p float='left'>
<img src="gif/ara/map3b.gif" width="49%" title="Anytime Search"> 
<img src="gif/rtaa/map3b.gif" width="49%" title="Agent Centered Search">
</p>
| map3c |
<p float='left'>
<img src="gif/ara/map3c.gif" width="49%" title="Anytime Search"> 
<img src="gif/rtaa/map3c.gif" width="49%" title="Agent Centered Search">
</p>
| map4 |
<p float='left'>
<img src="gif/ara/map4.gif" width="49%" title="Anytime Search"> 
<img src="gif/rtaa/map4.gif" width="49%" title="Agent Centered Search">
</p>
| map5 |
<p float='left'>
<img src="gif/ara/map5.gif" width="49%" title="Anytime Search"> 
<img src="gif/rtaa/map5.gif" width="49%" title="Agent Centered Search">
</p>
| map6 |
<p float='left'>
<img src="gif/ara/map6.gif" width="49%" title="Anytime Search"> 
<img src="gif/rtaa/map6.gif" width="49%" title="Agent Centered Search">
</p>

For Map7, we got the epsilon sub-optimal A* algorithm working but were not able to make the Agent Centered Search work properly.

| map7 |
<p float='center'>
<img src="gif/map7_special.gif" title="Anytime Search"> 
</p>
