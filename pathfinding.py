import sys
from collections import deque
from heapq import heappush, heappop

# Load the txt maze file into 2D list, as well as recording starting and ending point
def loadMaze(filename):
	f = open(filename, 'r')
	maze = []
	start = []
	end = []
	i = 0
	for line in f:
		maze.append([])
		j = 0
		for c in line:
			if c != '\n':
				maze[i].append(c)
				if c == 'P':
					start.append(i)
					start.append(j)
				if c == '.':
					end.append(i)
					end.append(j)
			j+=1
		i+=1
	
	f.close()
	return maze, start, end

# print the maze to stdout
def printMaze(maze):
	for line in maze:
		for c in line:
			print(c,end='')
		print('\n')

# display the solution
def displaySolution(maze, path, nodes, start, end):
	for point in path:
		maze[point[0]][point[1]] = '*'
	maze[start[0]][start[1]] = 'S'
	maze[end[0]][end[1]] = 'G'
	printMaze(maze)
	print('Path cost (number of steps): %g'%(len(path)-1))
	print('Number of nodes expanded: %g'%(len(nodes)))

# maze solver
def solver(filename, option, astar_h):
	maze, start, end = loadMaze(filename)
	#print start
	#print end
	if (start == end):
		return true
	solution = False
	nodes = [] # track nodes expanded
	frontier = [start] # track the frontier
	path = [] # track the actual path to the goal
	visited = [] # track the visited nodes
	if option == 'dfs':
		print('DFS:')
		dfs(maze, start, end, nodes, frontier, path, visited)
	if option == 'bfs':
		print('BFS:')
		bfs(maze, start, end, nodes, frontier, path, visited)
	if option == 'greedy':
		print('Greedy best first search:')
		greedy(maze, start, end, nodes, frontier, path, visited)
	if option == 'A':
		print('A star search:')
		Astar(maze, start, end, nodes, frontier, path, visited, astar_h)

# depth first search method
def dfs(maze, start, end, nodes, frontier, path, visited):
	found = False
	while (len(frontier) > 0 and found == False):
		# check if reaching the goal state
		if (frontier[len(frontier)-1] == end):
			path.append(frontier[len(frontier)-1])
			found = True
			break
		# if not reaching goal state, pop top element from the stack
		node = frontier.pop()
		nodes.append(node)
		path.append(node)
		if (node not in visited):
			visited.append(node)
		# find unvisited neighbors
		neighbors = findNeighbors(node, maze, visited)
		if (len(neighbors) > 0):
			for neighbor in neighbors:
				frontier.append(neighbor)
		else:
			# no neighbors, need to roll back to the node that has unvisited neighbors
			while(len(findNeighbors(path[len(path)-1], maze, visited)) == 0):
				path.pop()

	if (found == True):
		# display the solution
		displaySolution(maze, path, nodes, start, end)

# breadth first search method
def bfs(maze, start, end, nodes, frontier, path, visited):
	found = False
	frontier = deque(frontier)
	parents = {}
	parents[tuple(start)] = None
	while (len(frontier) > 0 and found == False):
		# check if reaching the goal state
		if (frontier[0] == end):
			path = getPath(frontier[0], parents)
			found = True
			break
		# if not reaching goal state, pop top element from the queue
		node = frontier.popleft()
		nodes.append(node)
		if (node not in visited):
			visited.append(node)
		# find unvisited neighbors
		neighbors = findNeighbors(node, maze, visited)
		if (len(neighbors) > 0):
			for neighbor in neighbors:
				if (neighbor not in frontier):
					frontier.append(neighbor)
					parents[tuple(neighbor)] = node

	if (found == True):
		# display the solution
		displaySolution(maze, path, nodes, start, end)

def getPath(node, parents):
	path = []
	while (node != None):
		path.append(node)
		node = parents[tuple(node)]
	return path

def greedy(maze, start, end, nodes, frontier, path, visited):
	found = False
	parents = {}
	parents[tuple(start)] = None
	pq = []
	heappush(pq, (manhattan(start, end), start))
	while (len(pq) > 0 and found == False):
		# check if reaching the goal state
		if (pq[0][1] == end):
			path = getPath(end, parents)
			found = True
			break
		# if not reaching goal state, pop top element from the priority queue
		node = heappop(pq)[1]
		nodes.append(node)
		if (node not in visited):
			visited.append(node)
		# find unvisited neighbors 
		neighbors = findNeighbors(node, maze, visited)
		if (len(neighbors) > 0):
			for neighbor in neighbors:
				if (manhattan(neighbor, end), neighbor) not in pq:
					heappush(pq, (manhattan(neighbor, end), neighbor))
					parents[tuple(neighbor)] = node

	if (found == True):
		# display the solution
		displaySolution(maze, path, nodes, start, end)

def Astar(maze, start, end, nodes, frontier, path, visited, astar_h):
	found = False
	parents = {}
	parents[tuple(start)] = None
	pq = []
	if astar_h == 'm':
		heuristic_fun = manhattan
	else:
		heuristic_fun = euclidean
	heappush(pq, (heuristic_fun(start, end), start))
	while (len(pq) > 0 and found == False):
		# check if reaching the goal state
		if (pq[0][1] == end):
			path = getPath(end, parents)
			found = True
			break
		# if not reaching goal state, pop top element from the priority queue
		node = heappop(pq)[1]
		nodes.append(node)
		if (node not in visited):
			visited.append(node)
		# find unvisited neighbors
		neighbors = findNeighbors(node, maze, visited)
		if (len(neighbors) > 0):
			for neighbor in neighbors:
				parents[tuple(neighbor)] = node
				if (AstarHeuristics(neighbor, end, parents, heuristic_fun), neighbor) not in pq:
					heappush(pq, (AstarHeuristics(neighbor, end, parents, heuristic_fun), neighbor))

	if (found == True):
		# display the solution
		displaySolution(maze, path, nodes, start, end)

def manhattan(node, end):
	return (abs(node[0] - end[0]) + abs(node[1] - end[1]))

def euclidean(node, end):
	return (((node[0] - end[0]) ** 2 + (node[1] - end[1]) ** 2) ** 0.5)

def AstarHeuristics(node, end, parents, h):
	return len(getPath(node, parents))-1 + h(node, end)

def findNeighbors(node, maze, visited):
	neighbors = []
	# down neighbor
	down = [node[0]+1, node[1]]
	if (down not in visited) and (maze[down[0]][down[1]] != '%'):
		neighbors.append(down)
	# left neighbor
	left = [node[0], node[1]-1]
	if (left not in visited) and (maze[left[0]][left[1]] != '%'):
		neighbors.append(left)
	# up neighbor
	up = [node[0]-1, node[1]]
	if (up not in visited) and (maze[up[0]][up[1]] != '%'):
		neighbors.append(up)
	# right neighbor
	right = [node[0], node[1]+1]
	if (right not in visited) and (maze[right[0]][right[1]] != '%'):
		neighbors.append(right)

	return neighbors

# Main function
def main():
	if (len(sys.argv) < 3):
		print("usage: python pathfinding.py option{dfs, bfs, greedy, A} mazefile")
		print('\n')
		sys.exit(1)
	option = sys.argv[1]
	mazefile = sys.argv[2]

	astar_h = 'm' # 'm' means manhattan, 'e' means Euclidean
	if (len(sys.argv) > 3):
		astar_h = sys.argv[3]

	solver(mazefile, option, astar_h)

if __name__ == '__main__':
	main()
