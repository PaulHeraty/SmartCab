import random
from environment import Agent, Environment
from planner import RoutePlanner
from simulator import Simulator
from collections import defaultdict
import datetime
import matplotlib.pyplot as plt

class LearningAgent(Agent):
    """An agent that learns to drive in the smartcab world."""

    def __init__(self, env):
        super(LearningAgent, self).__init__(env)  # sets self.env = env, state = None, next_waypoint = None, and a default color
        self.color = 'red'  # override color
        self.planner = RoutePlanner(self.env, self)  # simple route planner to get next_waypoint
        # TODO: Initialize any additional variables here
	self.Qvalues = defaultdict(int)
	self.successes = []
	#print "Successes {}".format(self.successes)
	self.num_moves = 0
	self.dist = 0
	self.rewards = 0

    def reset(self, destination=None):
        self.planner.route_to(destination)
        # TODO: Prepare for a new trip; reset any variables here, if required
	#print "Successes {}".format(self.successes)
	self.num_moves = 0
	self.dist = 0
	self.rewards = 0

    def distance(self, start, dest):
	if dest[0] > start[0]:
		xDist = min(abs(dest[0] - start[0]), abs(8 - dest[0] + start[0] - 1))
	else:
		xDist = min(abs(dest[0] - start[0]), abs(8 - start[0] + dest[0] - 1))
	if dest[1] > start[1]:
		yDist = min(abs(dest[1] - start[1]), abs(6 - dest[1] + start[1] - 1))
	else:
		yDist = min(abs(dest[1] - start[1]), abs(6 - start[1] + dest[1] - 1))
	#return abs(dest[0] - start[0]) + abs(dest[1] - start[1])
	return xDist + yDist

    def getBestAction(self):
	# Return the best action to take, i.e. one with highest Qscore
	leftScore = self.Qvalues[self.state + ('left',)]
	rightScore = self.Qvalues[self.state + ('right',)]
	forwardScore = self.Qvalues[self.state + ('forward',)]
	noneScore = self.Qvalues[self.state + (None,)]
	maxScore = max([leftScore, rightScore, forwardScore, noneScore])
	# Check to see if there is more than one maxScore
	indices = [i for i, x in enumerate([noneScore, forwardScore, leftScore, rightScore]) if x == maxScore]
	if len(indices) > 1:
		return maxScore, Environment.valid_actions[random.choice(indices)]
	elif leftScore == maxScore:
		return maxScore, 'left'
	elif rightScore == maxScore:
		return maxScore, 'right'
	elif forwardScore == maxScore:
		return maxScore, 'forward'
	else:
		return maxScore, None

    def update(self, t):
        # Gather inputs
        self.next_waypoint = self.planner.next_waypoint()  # from route planner, also displayed by simulator
        inputs = self.env.sense(self)
        deadline = self.env.get_deadline(self)

	# Calcaulate the distance
	location = self.env.agent_states[self]["location"]
	destination = self.env.agent_states[self]["destination"]
	if self.dist == 0:
		self.dist = self.distance(location, destination)

        # TODO: Update state
	self.state = (self.next_waypoint, inputs['light'], inputs['oncoming'], inputs['left'])
	#print "STATE: {}".format(self.state)
        
        # TODO: Select action according to your policy
        #action = None
        #action = random.choice(Environment.valid_actions)
	score, action = self.getBestAction()
	#print "  Best action {}  Best Score {}".format(action, score)

        # Execute action and get reward
	previous_state = self.state
        reward = self.env.act(self, action)
	if reward < 0.0:
		self.rewards += reward
		print "Negative reward: inputs = {}, action = {}, reward = {}, waypoint {}".format(inputs, action, reward, self.next_waypoint)
	
	# Increment num_moves unless action is None (as waiting at lights should not be penalized)
	if action != None:
		self.num_moves += 1
		#print "  Action {}  #Moves {}".format(action, self.num_moves)

        # TODO: Learn policy based on state, action, reward
	alpha = 0.5
	gamma = 0.1
        inputs = self.env.sense(self)
        future_waypoint = self.planner.next_waypoint()  # from route planner, also displayed by simulator
	self.state = (future_waypoint, inputs['light'], inputs['oncoming'], inputs['left'])
	maxNextStateQval, nextDir = self.getBestAction()
  	#print "  maxNextStateQval : {}".format(maxNextStateQval)
	#self.Qvalues[self.state + (action,)] = ((1-alpha) * self.Qvalues[self.state + (action,)]) + (alpha * (reward + (gamma * maxNextStateQval)))
	self.Qvalues[previous_state + (action,)] = self.Qvalues[previous_state + (action,)] + (alpha * (reward + (gamma * maxNextStateQval)) - self.Qvalues[previous_state + (action,)])
	#print "  UPDATE: {} = {}".format(self.state, self.Qvalues[self.state])

        #print "  LearningAgent.update(): deadline = {}, inputs = {}, action = {}, reward = {}".format(deadline, inputs, action, reward)  # [debug]

	# If we've reached the destination, then update succcesses
	location = self.env.agent_states[self]["location"]
	destination = self.env.agent_states[self]["destination"]
	if location==destination:
		efficiency = self.dist * 1./ self.num_moves
		rewards = self.rewards * 1. / self.num_moves
		#print "  Num Moves {}   Dist {}".format(self.num_moves, self.dist)
		self.successes.append([1., efficiency, self.rewards])


def run():
    """Run the agent for a finite number of trials."""

    # Set up environment and agent
    e = Environment()  # create environment (also adds some dummy traffic)
    a = e.create_agent(LearningAgent)  # create agent
    e.set_primary_agent(a, enforce_deadline=True)  # specify agent to track
    # NOTE: You can set enforce_deadline=False while debugging to allow longer trials

    # Now simulate it
    #sim = Simulator(e, update_delay=0.5, display=True)  # create simulator (uses pygame when display=True, if available)
    sim = Simulator(e, update_delay=0.005, display=False)  # create simulator (uses pygame when display=True, if available)
    # NOTE: To speed up simulation, reduce update_delay and/or set display=False

    n_trials = 100
    sim.run(n_trials)  # run for a specified number of trials
    # NOTE: To quit midway, press Esc or close pygame window, or hit Ctrl+C on the command-line

    # Report number of successes
    #print "Array {}".format(a.successes)
    successes = [item[0] for item in a.successes]
    print "Percentage of successful runs {}".format((sum(successes) / n_trials) * 100.0)

    # Report efficiencies
    efficiencies = [item[1] for item in a.successes]
    plt.plot(efficiencies)
    plt.ylabel('Efficiency')
    plt.xlabel('Run Number')
    plt.show()

    # Report rewards
    rewards = [item[2] for item in a.successes]
    plt.plot(rewards)
    plt.ylabel('Penalties')
    plt.xlabel('Run Number')
    plt.show()


if __name__ == '__main__':
    run()
