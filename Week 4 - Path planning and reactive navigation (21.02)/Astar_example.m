% A* planner example
clear

load("ocmap.mat")
show(ocmap)

start = [90, 90]
goal = [40, 40]

% A* planner
planner = plannerAStarGrid(ocmap)

plan(planner,start,goal);

show(planner)
