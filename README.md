# About
Unity project showcasing A* pathfinding, fully jobified &amp; burst compiled. The grid that is used to perform pathfinding is also built using jobs, with raycasting to scan the walkable floor and boxcasting to bake the obstacles. 

Pathfinding is performed for each agent, each frame. There is no local avoidance, so agents will overlap. A lot of improvements could be made, but the intention was to benchmark how many paths per frame my PC could handle at a reasonable framerate.

Remember to deactivate jobs leak detection and safety checks to avoid overhead.

# My benchmark
My laptop (i7-7700hq) can handle ~500 agents above 60 fps (at the start of the game). Note that as agents move and get closer to the goal, the computation time decreases, so if you are CPU bound, expect the framerate to increase as agents get closer to the end of the path.

I encourage you to make a build and play with different number of agents spawned.

# Preview
