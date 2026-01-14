import numpy as np
import random
from math import inf


class Particle:
    """
    Represents a particle of the Particle Swarm Optimization algorithm.
    """
    def __init__(self, lower_bound, upper_bound):
        """
        Creates a particle of the Particle Swarm Optimization algorithm.

        :param lower_bound: lower bound of the particle position.
        :type lower_bound: numpy array.
        :param upper_bound: upper bound of the particle position.
        :type upper_bound: numpy array.
        """
        # Todo: implement

        self.position = np.random.uniform(lower_bound,upper_bound) 
        self.velocity = np.random.uniform(-(upper_bound - lower_bound),upper_bound - lower_bound)
        # posso iniciar com velocidade de 0
        #self.velocity = np.zeros_like(self.position)
        self.best_cognitive_value = -inf
        self.cognitive_best_position = self.position
        # Particle Initialized (Position and Velocity)


class ParticleSwarmOptimization:
    """
    Represents the Particle Swarm Optimization algorithm.
    Hyperparameters:
        inertia_weight: inertia weight.
        cognitive_parameter: cognitive parameter.
        social_parameter: social parameter.

    :param hyperparams: hyperparameters used by Particle Swarm Optimization.
    :type hyperparams: Params.
    :param lower_bound: lower bound of particle position.
    :type lower_bound: numpy array.
    :param upper_bound: upper bound of particle position.
    :type upper_bound: numpy array.
    """
    def __init__(self, hyperparams, lower_bound, upper_bound):
        # Todo: implement

        self.num_particles = hyperparams.num_particles
        self.inertia_weight = hyperparams.inertia_weight
        self.cognitive_parameter = hyperparams.cognitive_parameter 
        self.social_parameter = hyperparams.social_parameter 

        self.lower_bound = lower_bound
        self.upper_bound = upper_bound
        self.velocity_max= upper_bound - lower_bound
        self.velocity_min = -self.velocity_max

        self.swarm = np.empty(self.num_particles,dtype=Particle)

        for k in range(self.num_particles):
            self.swarm[k] = Particle(lower_bound, upper_bound)
        # I initialized all the particles based on hyperparameters

        self.best_social_value = -inf
        self.best_social_position = np.copy(self.swarm[0].position) # Initializing best_postition
        self.interaction = 0 


    def get_best_position(self):
        """
        Obtains the best position so far found by the algorithm.

        :return: the best position.
        :rtype: numpy array.
        """
        # Todo: implement
        # If the new position has better self.best the 
        # we trade the position for ther better value

        return self.best_social_position

    def get_best_value(self):
        """
        Obtains the value of the best position so far found by the algorithm.

        :return: value of the best position.
        :rtype: float.
        """
        # Todo: implement
    
        return self.best_social_value

    def get_position_to_evaluate(self):
        """
        Obtains a new position to evaluate.

        :return: position to evaluate.
        :rtype: numpy array.
        """
        # Todo: implement

        return self.swarm[self.interaction].position 

    def advance_generation(self):
        """
        Advances the generation of particles. Auxiliary method to be used by notify_evaluation().
        """
        # Todo: implement    
        # I need to limit of the position for not run 

        rp = random.uniform(0,1)
        rq = random.uniform(0,1)
        

        cognitive_velocity = self.cognitive_parameter*rp*(-self.get_position_to_evaluate() + self.swarm[self.interaction].cognitive_best_position)
        social_velocity = self.social_parameter*rq*(-self.get_position_to_evaluate()+ self.get_best_position()) 
        self.swarm[self.interaction].velocity = self.swarm[self.interaction].velocity*self.inertia_weight + cognitive_velocity + social_velocity
        self.swarm[self.interaction].position += self.swarm[self.interaction].velocity  #posso colocar uma taxa de aprendizagem
        
        # BOUND THE POSITION
        self.swarm[self.interaction].position = np.minimum(np.maximum(self.get_position_to_evaluate(), self.lower_bound), self.upper_bound)
        # BOUND THE VELOCITY
        self.swarm[self.interaction].velocity = np.minimum(np.maximum(self.swarm[self.interaction].velocity, self.velocity_min),self.velocity_max)        

        self.interaction += 1
        self.interaction = self.interaction % self.num_particles


    def notify_evaluation(self, value):
        current_p = self.swarm[self.interaction]
        
        # Atualiza melhor global
        if value > self.best_social_value:
            self.best_social_value = value
            self.best_social_position = np.copy(current_p.position)

        # Atualiza melhor individual
        if value > current_p.best_cognitive_value:
            current_p.best_cognitive_value = value
            current_p.cognitive_best_position = np.copy(current_p.position)
        
        self.advance_generation()
