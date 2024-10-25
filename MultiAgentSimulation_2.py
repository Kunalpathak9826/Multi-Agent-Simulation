import time
import sys
import numpy as np
import csv
import heapq
from collections import deque
import matplotlib.pyplot as plt
import json
import cProfile
import pstats
import io


# Constants
TASK_IDLE = 0
TASK_ASSIGNED = 1
TASK_COMPLETED = 2

# Token class for holding state of the simulation
class Token:
    def __init__(self):
        self.agents = []
        self.path = []
        self.my_map = []
        self.my_endpoints = []
        self.tasks = []
        self.timestep = 0

class Endpoint:
    def __init__(self, loc=None):
        self.id = None
        self.loc = loc
        self.loc = loc if loc is not None else 131
        self.h_val = []

    def set_h_val(self, map_, col):
        self.h_val = self._bfs(map_, col)

    def _bfs(self, map_, col):
        Q = deque()
        status = [False] * len(map_)
        h = [-1] * len(map_)
        neighbors = [1, -1, col, -col]
        
        #print(f"self.loc: {self.loc}")
        status[self.loc] = True
        h[self.loc] = 0
        Q.append(self.loc)
        
        while Q:
            v = Q.popleft()
            for i in range(4):
                u = v + neighbors[i]
                if 0 <= u < len(map_) and map_[u]:  # check within bounds and if accessible
                    if not status[u]:
                        status[u] = True
                        h[u] = h[v] + 1
                        Q.append(u)
        
        return h

class Node:
    """Node class used for A* search."""
    def __init__(self, loc, g_val, parent=None, timestep=0, h_val=0, in_openlist=False):
        self.loc = loc
        self.g_val = g_val
        self.h_val = h_val
        self.parent = parent
        self.timestep = timestep
        self.in_openlist = in_openlist

    def get_f_val(self):
        return self.g_val + self.h_val

    def __lt__(self, other):
        if self.get_f_val() == other.get_f_val():
            return self.g_val > other.g_val
        return self.get_f_val() < other.get_f_val()

def AStar(start, goal, grid):
    def heuristic(a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    open_list = []
    start_node = Node(start, g_val=0, h_val=heuristic(start, goal))
    heapq.heappush(open_list, start_node)
    
    came_from = {}
    g_score = {start: 0}

    while open_list:
        current = heapq.heappop(open_list)

        if current.loc == goal:
            return reconstruct_path(came_from, current)

        for neighbor in get_neighbors(current.loc, grid):
            tentative_g_score = g_score[current.loc] + 1

            if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                g_score[neighbor] = tentative_g_score
                h_val = heuristic(neighbor, goal)
                neighbor_node = Node(neighbor, g_val=tentative_g_score, h_val=h_val, parent=current)
                came_from[neighbor] = current.loc
                heapq.heappush(open_list, neighbor_node)

    return []

def get_neighbors(pos, grid):
    neighbors = []
    x, y = pos
    for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
        new_x, new_y = x + dx, y + dy
        if 0 <= new_x < len(grid) and 0 <= new_y < len(grid[0]):
            if grid[new_x][new_y] != 1:
                neighbors.append((new_x, new_y))
    return neighbors

def reconstruct_path(came_from, current):
    path = []
    while current:
        path.append(current.loc)
        current = current.parent
    return path[::-1]

class Task:
    def __init__(self, start, goal, ts, tg):
        self.start = start
        self.goal = goal
        self.ts = ts
        self.tg = tg
        self.state = TASK_IDLE
        self.agent = None
        self.ag_arrive_start = -1
        self.ag_arrive_goal = -1

class Agent:
    def __init__(self, agent_id, col, row, loc, max_time):
        self.agent_id = agent_id
        self.path = [0] * max_time
        self.loc = loc
        self.finish_time = 0
        self.max_time = max_time
        self.position = (0, 0)  # Update based on map file
        self.task = None
        self.endpoints = []
        self.tasks_completed = []

    def set_loc(self, location):
        self.loc = location
        self.path = [location] * self.max_time

    def move_to(self, destination, grid):
        self.path = AStar(self.position, destination, grid)
        self.position = destination

    def totp(self, token):
        # Find available task for TOTP
        """Total Order Task Processing (TOTP) strategy."""
        for task in token.tasks:
            if task.state == TASK_IDLE and token.timestep >= task.ts:
                self.task = task
                task.state = TASK_ASSIGNED
                task.agent = self
                task.ag_arrive_start = token.timestep
                self.move_to((task.goal.loc // token.my_map.shape[1], task.goal.loc % token.my_map.shape[1]), token.my_map)
                return True
        return False

    def tptr(self, token):
        # Find task based on earliest start time for TPTR
        """Task Processing with Task Reassignment (TPTR) strategy."""
        available_tasks = [task for task in token.tasks if task.state == TASK_IDLE and token.timestep >= task.ts]
        if available_tasks:
            self.task = min(available_tasks, key=lambda t: t.ts)  # Take the earliest task
            self.task.state = TASK_ASSIGNED
            self.task.agent = self
            self.task.ag_arrive_start = token.timestep
            self.move_to((self.task.goal.loc // token.my_map.shape[1], self.task.goal.loc % token.my_map.shape[1]), token.my_map)
            return True
        return False
    
def BFS_find_nearest_empty(agent, grid):
    """Find the nearest empty endpoint using BFS."""
    q = deque([agent.position])
    visited = set()
    visited.add(agent.position)

    while q:
        current = q.popleft()
        if grid[current[0]][current[1]] == 0:  # Empty cell
            return current

        for neighbor in get_neighbors(current, grid):
            if neighbor not in visited:
                visited.add(neighbor)
                q.append(neighbor)

    return None  # No empty endpoint found

class Simulation:
    def __init__(self, map_name, task_name):
        self.computation_time = 0
        self.num_computations = 0
        self.token = Token()
        self.agents = []
        self.endpoints = []
        self.tasks = []
        self.row = 0
        self.col = 0
        self.maxtime = 0
        self.workpoint_num = 0
        self.t_task = 0
        self.load_map(map_name)
        self.load_task(task_name)

    def load_map(self, fname):
        try:
            with open(fname, 'r') as myfile:
                data = myfile.read()
                #json_data = re.search(r'{.*}', data, re.DOTALL).group(0)
                json_data = data[data.index('{'):data.rindex('}') + 1]
                map_info = json.loads(json_data)

                self.row = map_info['dimensions']['rows'] + 2
                self.col = map_info['dimensions']['cols'] + 2
                self.workpoint_num = map_info['workpoints']  # Number of workpoints
                agent_num = map_info['agents']  # Number of agents
                self.maxtime = map_info['max_time']  # Max timesteps

                # Initialize agents and endpoints
                self.agents = [Agent(i, self.col, self.row, i, self.maxtime) for i in range(agent_num)]
                self.token.agents = [None] * agent_num
                self.token.path = [[0] * self.maxtime for _ in range(agent_num)]
                self.endpoints = [Endpoint() for _ in range(self.workpoint_num + agent_num + 50)]
                self.token.my_map = np.zeros(self.row * self.col, dtype=bool)
                self.token.my_endpoints = np.zeros(self.row * self.col, dtype=bool)
                
                # Read map
                ep = 0
                ag = 0
                for i in range(1, self.row - 1):
                    line = map_info['layout'][i - 1]  # Assuming layout is a list of strings
                    for j in range(1, self.col - 1):
                        self.token.my_map[self.col * i + j] = (line[j - 1] != '@')
                        self.token.my_endpoints[self.col * i + j] = (line[j - 1] == 'e' or line[j - 1] == 'r')
                        if line[j - 1] == 'e':
                            self.endpoints[ep].loc = i * self.col + j
                            ep += 1
                        elif line[j - 1] == 'r':
                            self.endpoints[self.workpoint_num + ag].loc = i * self.col + j
                            self.agents[ag].set_loc(i * self.col + j)
                            self.token.agents[ag] = self.agents[ag]
                            self.token.path[ag] = [i * self.col + j] * self.maxtime
                            ag += 1
                        #print(f"ep: {ep}, len(self.endpoints): {len(self.endpoints)}")


                # Set borders
                for i in range(self.row):
                    self.token.my_map[i * self.col] = False
                    self.token.my_map[i * self.col + self.col - 1] = False
                for j in range(1, self.col - 1):
                    self.token.my_map[j] = False
                    self.token.my_map[self.row * self.col - self.col + j] = False

                # Initialize heuristic matrix for each endpoint
                for e, endpoint in enumerate(self.endpoints):
                    endpoint.set_h_val(self.token.my_map, self.col)
                    endpoint.id = e

        except FileNotFoundError:
            print("Map file not found.", file=sys.stderr)
            sys.exit()

    def load_task(self, fname):
        try:
            with open(fname, 'r') as myfile:
                reader = csv.reader(myfile)
                task_num = int(next(reader)[0])
                self.tasks = [[] for _ in range(self.maxtime)]
                for _ in range(task_num):
                    line = next(reader)
                    t_task, s, g, ts, tg = map(int, line)
                    self.tasks[t_task].append(Task(self.endpoints[s], self.endpoints[g], ts, tg))

                if len(self.tasks[0]) > 0:
                    for task in self.tasks[0]:
                        self.token.tasks.append(task)
        except FileNotFoundError:
            print("Task file not found.", file=sys.stderr)
            sys.exit()

    def run_totp(self):
        print("************TOTP************")
        while len(self.token.tasks) > 0 or self.token.timestep <= self.t_task:
            ag = self.agents[0]
            for agent in self.agents[1:]:
                if agent.finish_time == self.token.timestep:
                    ag = agent
                    break
                elif agent.finish_time < ag.finish_time:
                    ag = agent

            # Add new tasks
            for i in range(self.token.timestep + 1, ag.finish_time + 1):
                if len(self.tasks[i]) > 0:
                    self.token.tasks.extend(self.tasks[i])

            # Update timestep
            self.token.timestep = ag.finish_time
            ag.loc = ag.path[self.token.timestep]

            if len(self.token.tasks) == 0:
                ag.finish_time += 1
                continue

            self.num_computations += 1
            start_time = time.time()
            if not ag.totp(self.token):
                print("Not get a task.", file=sys.stderr)
                sys.exit()
            self.computation_time += time.time() - start_time

    def run_tptr(self):
        print("************TPTR************")
        while len(self.token.tasks) > 0 or self.token.timestep <= self.t_task:
            ag = self.agents[0]
            for agent in self.agents[1:]:
                if agent.finish_time == self.token.timestep:
                    ag = agent
                    break
                elif agent.finish_time < ag.finish_time:
                    ag = agent

            # Add new tasks to token
            for i in range(self.token.timestep + 1, ag.finish_time + 1):
                if len(self.tasks[i]) > 0:
                    self.token.tasks.extend(self.tasks[i])

            # Update timestep
            self.token.timestep = ag.finish_time
            ag.loc = ag.path[self.token.timestep]

            # Remove completed tasks
            self.token.tasks = [task for task in self.token.tasks if task.state != 'TAKEN' or self.token.timestep < task.ag_arrive_start]

            self.num_computations += 1
            start_time = time.time()
            if not ag.tptr(self.token):
                print("Not get a task.", file=sys.stderr)
                sys.exit()
            self.computation_time += time.time() - start_time

    def plot_grid(self):
        grid = np.zeros((self.row, self.col))
        for ep in self.endpoints:
            x = ep.loc // self.col
            y = ep.loc % self.col
            grid[x, y] = 2  # Mark endpoint locations
        
        for ag in self.agents:
            x = ag.loc // self.col
            y = ag.loc % self.col
            grid[x, y] = 1  # Mark agent locations
        
        plt.imshow(grid, cmap="hot", interpolation="nearest")
        #for agent in self.agents:
            #plt.plot(agent.position[1], agent.position[0], 'bo', markersize=10, label=f'Agent {agent.agent_id}')
        agent_plotted = False
        for agent in self.agents:
            if not agent_plotted:
                plt.plot(agent.position[1], agent.position[0], 'bo', markersize=10, label='Agents')
                agent_plotted = True
            else:
                plt.plot(agent.position[1], agent.position[0], 'bo', markersize=10)
       
        endpoint_plotted = False
        for endpoint in self.endpoints:
            if not endpoint_plotted:
                plt.plot(endpoint.loc % len(grid[0]), endpoint.loc // len(grid[0]), 'rx', markersize=8, label='Endpoints')
                endpoint_plotted = True
            else:
                plt.plot(endpoint.loc % len(grid[0]), endpoint.loc // len(grid[0]), 'rx', markersize=8)

        plt.title("Grid Visualization")
        plt.xlabel("X Coordinate")
        plt.ylabel("Y Coordinate")
        plt.legend(loc='right', bbox_to_anchor=(1.14, 0.5), prop={'size': 9})
        plt.grid()
        plt.show()

    def show_task(self):
        waiting_time = 0
        last_finish = 0
        print("TASK")
        for i, task_list in enumerate(self.tasks):
            for task in task_list:
                waiting_time += task.ag_arrive_goal - i
                last_finish = max(last_finish, task.ag_arrive_goal)
        print(f"Finishing Timestep: {last_finish}")
        print(f"Sum of Task Waiting Time: {waiting_time}")

    def save_task(self, fname, instance_name):
        with open(fname, 'a') as fout:
            waiting_time = 0
            last_finish = 0
            for i, task_list in enumerate(self.tasks):
                for task in task_list:
                    waiting_time += task.ag_arrive_goal - i
                    last_finish = max(last_finish, task.ag_arrive_goal)
            fout.write(f"{instance_name} {last_finish} {waiting_time} {self.computation_time / last_finish}\n")

    def save_path(self, fname):
        with open(fname, 'w') as fout:
            for agent_path in self.token.path:
                fout.write(f"{self.maxtime}\n")
                for j in range(self.maxtime):
                    x = agent_path[j] % self.col - 1
                    y = agent_path[j] // self.col - 1
                    fout.write(f"{x}\t{y}\n")

def read_agents_from_csv(file):
    """Read agent configurations from a CSV file."""
    agents = []
    with open(file, 'r') as myfile:
        reader = csv.DictReader(myfile)
        for row in reader:
            print(row) 
            #agent = Agent(int(row['AgentID']), int(row['MaxTime']), int(row['FinishTime']), int(row['Path']), int(row['Task']))
            agent_id = int(row[0])  # Assuming agent_id is the first column
            loc = row[1]  # Assuming loc is the second column (as a string)
            max_time = int(row[2])
            agents.append(Agent(agent_id, loc, max_time))
    return agents

def batch_run(input_files, output_file):
    """Batch run for multiple input files, appending each result to a single output file."""
    with open(output_file, 'a') as fout:
        for i, file in enumerate(input_files):
            # Load agents from the input file
            agents = read_agents_from_csv(file)
            
            # Initialize the simulation for each input file and execute it
            sim = Simulation(file.replace(), file)  # Adjust map file if required

            # Run COBRA approach (use run_totp() or run_tptr() based on the strategy)
            sim.run_totp()  # or sim.run_tptr() if preferred

            # Append results to the output file
            fout.write(f"Results for input file {i+1} ({file}):\n")
            sim.save_task(output_file, f"Instance_{i+1}")
            fout.write("\n")  # Add a newline for separation between results

            print(f"Completed batch run for {file}. Results appended to {output_file}.")


def main_loop(agents):
    """Main loop to simulate the COBRA approach."""
    tasks = []  # List of tasks to process

    # Example grid (5x5)
    grid = [
        [0, 1, 0, 0, 0],
        [0, 1, 0, 1, 0],
        [0, 0, 0, 1, 0],
        [1, 1, 0, 0, 0],
        [0, 0, 0, 0, 0]
    ]

    # Initialize endpoints and set their heuristic values
    col = len(grid[0])
    endpoints = [Endpoint(2), Endpoint(10)]
    for idx, endpoint in enumerate(endpoints):
        endpoint.id = idx
        endpoint.set_h_val([cell for row in grid for cell in row], col)  # Flatten the grid for the BFS map

    for agent in agents:
        task = agent.totp(agent, tasks, grid)
        if task:
            agent.move_to(task.start.loc, grid)  # Pass grid here
            agent.move_to(task.goal.loc, grid)    # Pass grid here
        else:
            nearest_empty_endpoint = BFS_find_nearest_empty(agent, grid)
            if nearest_empty_endpoint:
                agent.move_to(nearest_empty_endpoint, grid)  # Pass grid here

    plot_grid(grid, agents, endpoints)  # Plot the grid after processing


def main():
    sim = Simulation('/Users/kunalpathak9826/Desktop/kiva-10-500-5_map.json', '/Users/kunalpathak9826/Desktop/kiva-0.5_task_2.csv')
    profiler = cProfile.Profile()
    profiler.enable()

    sim.run_totp()
    #sim.run_tptr()  

    profiler.disable()
    result = io.StringIO()
    ps = pstats.Stats(profiler, stream=result).sort_stats(pstats.SortKey.TIME)
    ps.print_stats()
    print(result.getvalue())
     
    sim.plot_grid()

    # Alternatively, to batch run
    input_files = [
        '/Users/kunalpathak9826/Desktop/untitled folder/kiva-0.2_task.csv',
        '/Users/kunalpathak9826/Desktop/untitled folder/kiva-0.5_task_2.csv',
        '/Users/kunalpathak9826/Desktop/untitled folder/kiva-1_task.csv',
        '/Users/kunalpathak9826/Desktop/untitled folder/kiva-2_task.csv',
        '/Users/kunalpathak9826/Desktop/untitled folder/kiva-5_task.csv'
    ]
    output_file = '/Users/kunalpathak9826/Desktop/batch_results.txt'
    batch_run(input_files, output_file)
    print(f"Batch processing completed. Consolidated output is saved in {output_file}.")

if __name__ == "__main__":
    main()
