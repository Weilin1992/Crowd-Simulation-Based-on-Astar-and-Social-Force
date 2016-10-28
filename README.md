#Crow Simulation using social force and A*


we combine the path-finding and social force algorithms together and create a crowd simulator. 
First in the preprocess part, we run the A* algorithm and get the path to the goal, and we add it to the goalqueue data structure. Then sfAI will set the nearest goal in the goalqueue that has direct eyesight of the agent as the current_goal for this agent. After the agent reaches current_goal, sfAI will find the next candidate for current_goal until simulation is finished. Finally, as we notice that some agents will deviate from the calculated path and get stuck due to the force from other agents, a recalculation part is added in updateAI() every 200 steps to ensure that agents will re-discover the optimal path to the final goal even if it gets stuck.




