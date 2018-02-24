import numpy as np
from sklearn.neighbors import KDTree

### 1. Monotonically Increasing Logistic function:
def log_inc(vec, param):     
    
    return vec**2/(vec**2 +param**2)

### 2. Monotonically decreasing logistic function: 
def log_dec(vec, param): 
    
    return param**2/(vec**2 + param**2)


### 3. This function returns the rho and theta parameters for the line: Theta is in radians and rho in pixels:
def get_line_eqn(p1,p2): 
    
    # First compute theta: tan(theta) = (x2 -x1)/(y1 -y2)
    # NOTE: theta is in radians
    theta = np.arctan2( (p2[0] - p1[0]), (p1[1] - p2[1]))

    # Now compute rho: 
    rho = p1[0]*np.cos(theta) + p1[1]*np.sin(theta)
    
    # Check and see if the distance from origin is a negative value:
    if rho < 0: 
        
        rho, theta = get_line_eqn(p2,p1)
    
    return (rho, theta)

### 4. Get the normal distance of a point from the line. 
## Logic: Rotate the query point into the co-ordinate system of the line. The new x cordinate subtracted from the normal 
#         distance (rho) gives us the normal distance of the point from the line.
def get_normal_dist(line, point): 
    
    """
    line - Tuple containing the (rho, theta) parameters of the line
    point - Tuple containing the (x,y) co-ods of the line
    """
    
    # Rotate: 
    x_rot = np.cos(line[1])*point[0] + np.sin(line[1])*point[1]
    
    # Normal distance: x_rot - rho:
    return x_rot - line[0]

### 5. Informed Sampling Function: 

# Logic: Samples pairs of points that are close to each other in order to ensure that true lines are sampled more often: 

def info_rand_sample_fit_lines(points, numLines): 
    
    # Find the nearest neighbors of all the points in the dataset: Using a kd-tree here:
    trainData = np.asarray(points)
    kdt = KDTree(trainData, leaf_size=30, metric='euclidean')
    
    # Query the kd-tree: 
    allDistances, indices = kdt.query(trainData, k=6, return_distance = True)
    
    
    # List of the pairs of indices:
    idxs = []
    
    # Loop through and generate pairs of indices in order to fit lines: 
    for count in range(numLines): 
        
        idx1 = np.random.choice(len(points))
        
        choiceIdx2 = indices[idx1, 1:]
        
        idx2 = np.random.choice(choiceIdx2)
        
        idxs.append((idx1, idx2))
        
    
    # Initialize a list for the fitted lines: 
    fitted_lines = []
    
    # Loop through the indices returned: 
    for idx in idxs:
        
        p1 = points[idx[0]]
        p2 = points[idx[1]]
        
        fitted_lines.append(get_line_eqn(p1,p2))        
    
    return fitted_lines, idxs

### Description: The steps in this function are: 
# 1. Randomly sample pairs of points (number of pairs is a parameter) - With replacement ??
# 2. Fit lines through the pairs of points. 
# 3. Compute normal distance of each point from each line - Nested loop operation. 
# 4. Compute Probability of membership of each point in each line
# 5. Iteratively refine estimates of: 
#    - Prob validity points 
#    - Prob validity lines

def iterative_function(points, numLines, numIter, e_tilde, gamma_tilde, beta_tilde): 
    
    """"
    Points - List of tuples, where each tuple has (x,y) co-ods of the points. 
    numLines - Number of pairs of points to be randomly sampled
    numIter - Number of ietrations for which estimates of Prob should be refined
    e_tilde - Critical distance for 50% probability of memebership in the line
    gamma_tilde - Critical fraction of valid points for 50% probability of validity of a line
    beta_tilde  - Critical fraction of valid lines for 50% probability of validity of a point
    
    """
    
    numPoints = len(points)
    
    # Randomly sample pairs and get the corresponding rho and theta parameters for a line fitted to the pair: 
    # Returns a list of tuples - Each tuple has the rho and theta parameters for the line: 
    lines ,idxs = info_rand_sample_fit_lines(points, numLines)
    
    
    # Compute normal distance of each point from each line: Store in a 2-D numpy array: 
    # Points along 1st axis - Rows - axis= 0
    # Lines along 2nd axis - Columns - axis=1
    
    # Initialize the 2-D array: 
    normDist = np.zeros((numPoints, numLines))
    
    
    # Indices for the 2-D array: 
    j,k = 0,0
    
    # Loop through points:
    for point in points: 
        
        k = 0
        
        # Loop through the lines: 
        for line in lines:
            
            normDist[j,k] = get_normal_dist(line,point)
            
            # Increment the column (line) index:
            k+=1
        
        
        #Increment the row (point) index
        j += 1
        
    # Transform the Normal Distance matrix to the Probability of Membership matrix: 
    Pr_C = log_dec(normDist,e_tilde)
    
    
    ## Iteratively refine estimates of Prob of Validity - Points and Lines: 
    iterCount = 0
    
    # Initialize Probability of Validity of points and lines: 
    initProb =1
    Pr_A = initProb*np.ones((numPoints,1))
    Pr_V = np.zeros((numLines,1))
    
    
    # Initialize gamma and beta: Fractions of valid points and lines respectively: 
    gamma = np.zeros_like(Pr_V)
    beta = np.zeros_like(Pr_A)
    
    
    while iterCount < numIter: 
        
        # For each line: 
        for k in range(numLines):
            
            # Compute expected fraction of valid points: 
            gamma[k] = np.dot(Pr_A.T, Pr_C[:,k])/np.sum(Pr_A)
            
            #print (gamma[k], end=" ->")
        
        # Compute Probability of Validity: 
        Pr_V = log_inc(gamma, gamma_tilde)
            
            
        
        # For each point: 
        for j in range(numPoints):
        
            # Compute expected fraction of valid lines in which it is a member: 
            beta[j] = np.dot(Pr_V.T, Pr_C[j,:])/np.sum(Pr_V)
            
            
            #print (beta[j], end=" ->")
            
        #print (" ")
            
        # Compute Probability of Validity: 
        Pr_A  = log_inc(beta, beta_tilde)
        
        
        iterCount +=1
        
    # Sort the lines according to Probability of Validity:
    idx_sort = np.argsort(Pr_V, axis=0)
    
    print (" The equations of candidate lines and their probability of validity are: ")   

    
    for idx in idx_sort:         
        print (lines[int(idx)] , end = '-- >')
        print (Pr_V[idx])
        
    return lines, Pr_A, Pr_V



### Description: The steps in this function are: 
# 1. Randomly sample pairs of points (number of pairs is a parameter) - With replacement ??
# 2. Fit lines through the pairs of points. 
# 3. Compute normal distance of each point from each line - Nested loop operation. 
# 4. Compute Probability of membership of each point in each line
# 5. Iteratively refine estimates of: 
#    - Prob validity points 
#    - Prob validity lines


## Need to work on the vectorization in more detail!!! 

def iterative_function_vect(points, numLines, numIter, e_tilde, gamma_tilde, beta_tilde): 
    
    """"
    Points - List of tuples, where each tuple has (x,y) co-ods of the points. 
    numLines - Number of pairs of points to be randomly sampled
    numIter - Number of ietrations for which estimates of Prob should be refined
    e_tilde - Critical distance for 50% probability of memebership in the line
    gamma_tilde - Critical fraction of valid points for 50% probability of validity of a line
    beta_tilde  - Critical fraction of valid lines for 50% probability of validity of a point
    
    """
    
    numPoints = len(points)
    
    # Randomly sample pairs and get the corresponding rho and theta parameters for a line fitted to the pair: 
    # Returns a list of tuples - Each tuple has the rho and theta parameters for the line: 
    lines ,idxs = info_rand_sample_fit_lines(points, numLines)
    
    
    # Compute normal distance of each point from each line: Store in a 2-D numpy array: 
    # Points along 1st axis - Rows - axis= 0
    # Lines along 2nd axis - Columns - axis=1
    
    # Initialize the 2-D array: 
    normDist = np.zeros((numPoints, numLines))
    
    
    # Indices for the 2-D array: 
    j,k = 0,0
    
    # Loop through points:
    for point in points: 
        
        k = 0
        
        # Loop through the lines: 
        for line in lines:
            
            normDist[j,k] = get_normal_dist(line,point)
            
            # Increment the column (line) index:
            k+=1
        
        
        #Increment the row (point) index
        j += 1
        
    # Transform the Normal Distance matrix to the Probability of Membership matrix: 
    Pr_C = log_dec(normDist,e_tilde)
    
    
    ## Iteratively refine estimates of Prob of Validity - Points and Lines: 
    iterCount = 0
    
    # Initialize Probability of Validity of points and lines: 
    initProb =1
    Pr_A = initProb*np.ones((numPoints,1))
    Pr_V = np.zeros((numLines,1))
    
    
    # Initialize gamma and beta: Fractions of valid points and lines respectively: 
    gamma = np.zeros_like(Pr_V)
    beta = np.zeros_like(Pr_A)
    
    
    while iterCount < numIter: 
        
        # For each line: Compute Gamma:
            
        # Compute expected fraction of valid points: 
        gamma = np.dot(Pr_A.T, Pr_C)/np.sum(Pr_A)  # Hope the broadcasting works here:
            
        
        # Compute Probability of Validity: 
        Pr_V = log_inc(gamma, gamma_tilde)           
            
        
        # For each point: Compute beta:
        
        # Compute expected fraction of valid lines in which it is a member: 
        beta = np.dot(Pr_C, Pr_V.T)/np.sum(Pr_V)
            
            
        # Compute Probability of Validity: 
        Pr_A  = log_inc(beta, beta_tilde)
        
        
        iterCount +=1
        
    # Sort the lines according to Probability of Validity:
    idx_sort = np.argsort(Pr_V, axis=1)
    
    print (" The equations of candidate lines and their probability of validity are: ")   

    
    for idx in idx_sort:         
        print (lines[int(idx)] , end = '-- >')
        print (Pr_V[idx])
        
    return lines, Pr_A, Pr_V