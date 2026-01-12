from math import inf


def hill_climbing(cost_function, neighbors, theta0, epsilon, max_iterations):
    """
    Executes the Hill Climbing (HC) algorithm to minimize (optimize) a cost function.

    :param cost_function: function to be minimized.
    :type cost_function: function.
    :param neighbors: function which returns the neighbors of a given point.
    :type neighbors: list of numpy.array.
    :param theta0: initial guess.
    :type theta0: numpy.array.
    :param epsilon: used to stop the optimization if the current cost is less than epsilon.
    :type epsilon: float.
    :param max_iterations: maximum number of iterations.
    :type max_iterations: int.
    :return theta: local minimum.
    :rtype theta: numpy.array.
    :return history: history of points visited by the algorithm.
    :rtype history: list of numpy.array.
    """
    theta = theta0
    history = [theta0]
    # Todo: Implement Hill Climbing
    
    theta = theta0
    history = [theta0]
    num_interactions = 0 #PODE SER UM A MAIS, DEPOIS PENSO
    custo = cost_function(theta)

    while custo > epsilon and num_interactions < max_iterations:
        num_interactions += 1
        custo = cost_function(theta)
        var = theta #Aqui só foi para inicializar

        for test in neighbors(theta):
            custo_temp = cost_function(test)

            if custo_temp < custo:
                custo = custo_temp
                var = test
        
        theta = var
        history.append(theta)              
        
    return theta, history
