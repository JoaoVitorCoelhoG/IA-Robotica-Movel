def gradient_descent(cost_function, gradient_function, theta0, alpha, epsilon, max_iterations):
    """
    Executes the Gradient Descent (GD) algorithm to minimize (optimize) a cost function.

    :param cost_function: function to be minimized.
    :type cost_function: function.
    :param gradient_function: gradient of the cost function.
    :type gradient_function: function.
    :param theta0: initial guess.
    :type theta0: numpy.array.
    :param alpha: learning rate.
    :type alpha: float.
    :param epsilon: used to stop the optimization if the current cost is less than epsilon.
    :type epsilon: float.
    :param max_iterations: maximum number of iterations.
    :type max_iterations: int.
    :return theta: local minimum.
    :rtype theta: numpy.array.
    :return history: history of points visited by the algorithm.
    :rtype history: list of numpy.array.
    """
    # Todo: Implement Gradient Descent
 
    theta = theta0
    history = [theta0]
    num_interactions = 0 #PODE SER UM A MAIS, DEPOIS PENSO
    custo = cost_function(theta)

    while custo > epsilon and num_interactions < max_iterations:
        num_interactions += 1
        theta = theta - alpha*gradient_function(theta)
        custo = cost_function(theta)
        history.append(theta)      

    return theta, history
