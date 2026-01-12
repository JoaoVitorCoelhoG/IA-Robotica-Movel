import random
import math
from constants import *

## NÃO SETEI VELOCIDADE

class FiniteStateMachine(object):
    """
    A finite state machine.
    """
    def __init__(self, state):
        self.state = state 

    def change_state(self, new_state):
        self.state = new_state

    def update(self, agent):
        self.state.check_transition(agent, self)
        self.state.execute(agent)


class State(object):
    """
    Abstract state class.
    """
    def __init__(self, state_name):
        """
        Creates a state.

        :param state_name: the name of the state.
        :type state_name: str
        """
        self.state_name = state_name

    def check_transition(self, agent, fsm):
        """
        Checks conditions and execute a state transition if needed.

        :param agent: the agent where this state is being executed on.
        :param fsm: finite state machine associated to this state.
        """
        raise NotImplementedError("This method is abstract and must be implemented in derived classes")

    def execute(self, agent):
        """
        Executes the state logic.

        :param agent: the agent where this state is being executed on.
        """
        raise NotImplementedError("This method is abstract and must be implemented in derived classes")


class MoveForwardState(State):
    def __init__(self):
        super().__init__("MoveForward")
        # Todo: add initialization code
        self.time = 0

    def check_transition(self, agent, state_machine):
        # Todo: add logic to check and execute state transition

	    # Agent é um Roomba e tem todas as funções de um Roomba
	    # state_machine é um FiniteStateMachine
                
        #SE ESTÁ BATENDO TROCAR O ESTADO PARA GoBackState
        if agent.bumper_state :
            agent.bumper_state = False
            state_machine.change_state(GoBackState())
            agent.behavior = state_machine
            return
        
        #Se o tempo está maior que T2, se mover em espiral
        if self.time > MOVE_FORWARD_TIME:
            self.time = 0	
            state_machine.change_state(MoveInSpiralState())
            agent.behavior = state_machine
            return
        #Se não está em nenhuma das condições anteriores, se move em frente
        self.time += SAMPLE_TIME
    

    def execute(self, agent):
        # Todo: add execution logic
        agent.set_velocity( FORWARD_SPEED, 0)

class MoveInSpiralState(State):
    def __init__(self):
        super().__init__("MoveInSpiral")
        self.time = 0
        # Todo: add initialization code
    
    def check_transition(self, agent, state_machine):
        # Todo: add logic to check and execute state transition
        #SE ESTÁ BATENDO TROCAR O ESTADO PARA GoBackState
        if agent.bumper_state :
            agent.bumper_state = False
            state_machine.change_state(GoBackState())
            agent.behavior = state_machine
            return
        if self.time > MOVE_IN_SPIRAL_TIME:
            self.time = 0	
            state_machine.change_state(MoveForwardState())
            agent.behavior = state_machine
            return
        self.time += SAMPLE_TIME
    
    def execute(self, agent):
        # Todo: add execution logic
        self.radius = INITIAL_RADIUS_SPIRAL + SPIRAL_FACTOR * self.time
        agent.set_velocity(FORWARD_SPEED, FORWARD_SPEED/self.radius)

class GoBackState(State):
    def __init__(self):
        super().__init__("GoBack")
        self.time = 0
        # Todo: add initialization code

    def check_transition(self, agent, state_machine):
        # Todo: add logic to check and execute state transition
        # Andar um tempo menor que T3

        if self.time > GO_BACK_TIME:
            self.time = 0	
            state_machine.change_state(RotateState())
            agent.behavior = state_machine
            return
        self.time += SAMPLE_TIME


    def execute(self, agent):
        agent.set_velocity(BACKWARD_SPEED, 0)


class RotateState(State):
    def __init__(self):
        super().__init__("Rotate")
        # Todo: add initialization code
        self.time = 0
        self.angulo = random.uniform(-math.pi, math.pi)
        self.tempo_girar = self.angulo/ANGULAR_SPEED

    def check_transition(self, agent, state_machine):
        if self.time > self.tempo_girar:
            self.time = 0	
            state_machine.change_state(MoveForwardState())
            agent.behavior = state_machine
        self.time += SAMPLE_TIME
    
    def execute(self, agent):
        agent.set_velocity(0, ANGULAR_SPEED)
