# About
Unity project showcasing A* pathfinding, fully jobified &amp; burst compiled. The grid that is used to perform pathfinding is also built using jobs, with raycasting to scan the walkable floor and boxcasting to bake the obstacles. 

Pathfinding is performed for each agent, each frame. It uses four neighbours and no path smoothing / postprocess is done. There is no local avoidance, so agents will overlap. Agents are moved using the IJobParallelForTransform. A lot of improvements could be made, but the intention was to benchmark how many paths per frame my PC could handle at a reasonable framerate.

Remember to keep jobs leak detection and safety checks deactivated to avoid overhead. Showing the rendering of the grid has also a minor impact on performance.

# Benchmark
The laptop I used (i7-7700hq) can handle ~500 agents above 60 fps (at the start of the game) in the provided scene (in build). Note that as agents move and get closer to the goal, the computation time decreases, so if you are CPU bound, expect the framerate to increase as agents get closer to the end of the path.

# Preview
![alt-text](./GithubImgs/TeaserGif.gif)

Grid build, agents pathfinding and movement profiling:
![alt-text](./GithubImgs/Profiler.png)
