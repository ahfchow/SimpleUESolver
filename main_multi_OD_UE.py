
'''
Created on 20 Jul 2018
- A code written for calculating network user equilibrium solution  
- tested on a simple network with single O-D connected by three parallel routes 
- (see Sheffi, 1985; p114, Aug 2018)

Developments: 
- solution algorithm based upon Sheffi (1985), page 119-120 (Aug 2018)
- extended to networks with multiple ODs (Sep 2018)
- included calculation of total network cost (Feb 2019)


@author: Andy Chow at CityU HK 
Revised: Jul 2020


'''
import numpy as np


# BPR cost function 
def BPR_function(f,tf,cap):
    return tf*(1+0.15*(f/cap)**4)

# Integral of BPR cost function (for Beckmann's transform; Step 3)
def In_BPR_function(f,tf,cap):
    return tf*f+0.15/5*tf*(f**5)*(1/cap)**4


# Deriative of BPR cost function w.r.t flow (for calculating SO, Feb 2019) 
def d_BPR_function(f,tf,cap):
    return (tf*0.15*(1/cap)**4)*4*f**3



# Converting path flows to link flows 
def PathToLink_f(fp,tf,LinkSeq):  
    Link_f = np.zeros(len(tf))     # link flow     
    
    for p in list(range(0,len(LinkSeq))):
        for i in list(range(0,len(LinkSeq[p]))):
            if LinkSeq[p][i] > 0:
                Link_f[int(LinkSeq[p][i])-1] = Link_f[int(LinkSeq[p][i])-1] + fp[p]     
                
    return Link_f        



# Path cost (and Link flow) calculation 
def PathCost(fp,tf,cap,LinkSeq):
    PathCost = []                  # path cost 
    
    # Converting path flows to link flows 
    Link_f = PathToLink_f(fp,tf,LinkSeq)
        
    # update of path costs 
    for p in list(range(0,len(LinkSeq))):
        cost = 0
        for i in list(range(0,len(LinkSeq[p]))):
            if LinkSeq[p][i] > 0:
                a = int(LinkSeq[p][i])-1    # link id in Python (starting from zero)
                cost = cost+BPR_function(Link_f[a],tf[a],cap[a])
                del a
        PathCost.append(cost)
    return PathCost



# searching shortest path (by sorting) 
def ShortestPath(PathC,OD_index,LinkSeq):
    
    LeastPathCost = 99999*np.ones(int(OD_index[-1]))   # cost on least cost path
    LeastPath = -1*np.ones(int(OD_index[-1]))          # path index for least cost path 
    for od in list(range(0,int(OD_index[-1]))):
        for p in list(range(0,len(LinkSeq))):
            if OD_index[p] == od+1:                    # OD id in Python (starting from zero)
                if PathC[p] <= LeastPathCost[od]:
                    LeastPathCost[od] = PathC[p]
                    LeastPath[od] = p
    return LeastPath, LeastPathCost


# Golden section line search (Step 3) 
def golden_section(lower, upper, merror, tf, fp, fp_dir):
    seg = (np.sqrt(5)-1)/2
    error = 1000

    while error >= merror:
        temp1 = upper-seg*(upper-lower)
        temp2 = lower+seg*(upper-lower)
        
        
        Link_f1 = PathToLink_f(fp+temp1*(fp_dir-fp),tf,LinkSeq)
        f1 = sum(In_BPR_function(Link_f1,Linktf,LinkCap))
        
        Link_f2 = PathToLink_f(fp+temp2*(fp_dir-fp),tf,LinkSeq)
        f2 = sum(In_BPR_function(Link_f2,Linktf,LinkCap))
        
        if f1-f2<0:
            upper = temp2
        else:
            lower = temp1
        error = np.abs(f1-f2)
        
    return (temp2+temp1)/2




# MAIN SCRIPT
# ------------

# Importing network setting
# importing link configuration
InArray = np.loadtxt('NetworkConfig_multi.csv',delimiter=',')
LinkFr = InArray.transpose()[0]   # Link 'From Node'
LinkTo = InArray.transpose()[1]   # Link 'To Node'
Linktf = InArray.transpose()[2]   # link free-flow travel time (BPR function)
LinkCap = InArray.transpose()[3]  # link capacity (BPR function)

del InArray


# importing OD matrix
InArray = np.loadtxt('OD_multi.csv',delimiter=',')
Origin = InArray.transpose()[0]        # Link 'From Node'
Destination = InArray.transpose()[1]   # Link 'To Node'
Demand = InArray.transpose()[2]        # link free-flow travel time (BPR function)

del InArray


# importing path sets 
InArray = np.loadtxt('Paths_multi.csv',delimiter=',')
OD_index = InArray.transpose()[0]    # which OD pair the path is collecting 
LinkSeq = []
for i in list(range(0,len(OD_index))):
    LinkSeq.append(InArray[i,1:])

del InArray




## Step 0: Initialisation / all-or-nothing assignment 
# initialising decision variables
fp = np.zeros([len(LinkSeq)])    # path inflows


# calculating initial path cost (with zero path inflows)
# PathC = PathCost(fp,Linktf,LinkCap,LinkSeq)         # for UE 
PathC = SocialPathCost(fp,Linktf,LinkCap,LinkSeq)   # for SO
  
# determining path with least cost (shortest path)
LeastPath, LeastPathCost = ShortestPath(PathC,OD_index,LinkSeq)

# assigning all demand flows to the shortest path 
for od in list(range(0,int(OD_index[-1]))):
    if int(OD_index[-1]) > 1:
        fp[int(LeastPath[od])] = Demand[od]
    else: 
        fp[int(LeastPath[od])] = Demand


k = 1   # iteration counter for UE solution
alpha = 1
conv = 99999 
# while conv > 0.01:
while k < 500:
    
    ## Step 1: Update path costs
    PathC = PathCost(fp,Linktf,LinkCap,LinkSeq)   # for UE 

    
    ## Step 2: Direction finding (fp_dir)
    # determining path with least cost (shortest path)
    fp_dir = np.zeros([len(LinkSeq)])    # auxiliary path inflows
    LeastPath, LeastPathCost = ShortestPath(PathC,OD_index,LinkSeq)
    
    for od in list(range(0,int(OD_index[-1]))):
        if int(OD_index[-1]) > 1:
            fp_dir[int(LeastPath[od])] = Demand[od]
        else: 
            fp_dir[int(LeastPath[od])] = Demand
    
    
    ## Step 3: Line search for step size and hence new path inflows given the auxiliary flow
    
    # determining step size using golden section line search
    lower = 0; upper = 1; merror = 0.0001
    alpha = golden_section(lower, upper, merror, Linktf, fp, fp_dir)
    
    
    
    ## Step 4: Update path inflow
    old_fp = fp
    old_Link_f = PathToLink_f(fp,Linktf,LinkSeq)
    
    fp = fp+alpha*(fp_dir-fp)
    Link_f = PathToLink_f(fp,Linktf,LinkSeq)
    
    ## Step 5: Convergence check 
    conv = Link_f - old_Link_f 
    conv = np.linalg.norm(conv,2)
    conv = conv/sum(old_Link_f)
     
    # print(conv, alpha)
    
    k += 1


# output final UE solution 
print(fp)
PathC = PathCost(fp,Linktf,LinkCap,LinkSeq)
print(PathC)


# calculate total network cost (Feb 2019)
TotalCost = np.dot(fp,PathC)
print(TotalCost)



 





















































