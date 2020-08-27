# SimpleUESolver

This is a simple user equilibrium traffic assignment solver for general network with multiple origins and destinations connected by general network with explicitly enumerated paths. The computation is coded by using Frank-Wolfe algorithm described in Sheffi (1985, Chapter 5). 

- Reference: 
Sheffi, Y. (1985) Urban Transportation Networks. Prentice-Hall, Inc. Englewood Cliffs, NJ. [http://web.mit.edu/sheffi/www/selectedMedia/sheffi_urban_trans_networks.pdf]


## Input files: 
1. NetworkConfig_multi.csv 
- Link set in the network (specified by BPR function): 
 
  - 1st Column: 'starting node' of the link 
  - 2nd Column: 'ending node' of the link 
  - 3rd Column: 'free-flow travel time' of the link 
  - 4th Column: 'capacity' of the link 


2. OD_multi.csv 
- Origin-destination (OD) pairs in the network: 

  - 1st Column: 'origin' of the OD pair
  - 2nd Column: 'destination' of the OD pair 
  - 3rd Column: 'demand' of the OD pair 


3. Paths_multi.csv
- Path set in the network: 

  - 1st Column: 'OD-pair' the path is connecting 
  - 2nd Column onward: Link sequence of the path (with '0' in the ending columns indicating end of the path)



## Output (on screen) 
1. path flows 
2. path costs 




