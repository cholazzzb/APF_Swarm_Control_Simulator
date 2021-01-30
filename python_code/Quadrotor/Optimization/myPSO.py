class myPSO(object):
    def __init__(self, particles, model, cost_function, target):
        self.model = model
        self.cost_function = cost_function

    def calculateBestParameter(self, w, c1, c2, totalParameters, particles, lowerBoundary, upperBoundary, total_iteration):
        # Setup the initial value
        if isMinimize:
            particle_fit_value = np.ones(particles) * 1e100
            global_fit_value = 1e100
        else:
            particle_fit_value = np.ones(particles) * 0
            global_fit_value = 0

        particle_position = np.random.uniform(
            lowerBoundary, upperBoundary, (particles, totalParameters))
        particle_position_best = particle_position
        particle_velocity = np.zeros((particles, totalParameters))
        global_best_position = np.zeros(totalParameters)

        # First loop
        for particle_index in range(particles):
            # Count
            cost_value = self.cost_function(
                particle_position[particle_index], target)

            # Update
            if(particle_fit_value[particle_index] > cost_value):
                particle_fit_value[particle_index] = cost_value
                particle_position_best[particle_index] = np.copy(
                    particle_position[particle_index])

            if(global_fit_value > cost_value):
                global_fit_value = cost_value
                global_best_position = np.copy(
                    particle_position[particle_index])

        # Second loop
        iterate = 0
        while (iterate < total_iteration):
            for particle_index in range(particles):
                # Count
                particle_velocity[particle_index] = w*particle_velocity[particle_index] + c1*random.random()*(
                    particle_position_best[particle_index]-particle_position[particle_index]) + c2*random.random()*(global_best_position-particle_position[particle_index])

                particle_position[particle_index] = particle_velocity[particle_index] + \
                    particle_position[particle_index]

                cost_value = self.cost_function(
                    particle_position[particle_index], target)

                # Update
                if(particle_fit_value[particle_index] > cost_value):
                    particle_fit_value[particle_index] = cost_value
                    particle_position_best[particle_index] = np.copy(
                        particle_position[particle_index])

                if(global_fit_value > cost_value):
                    global_fit_value = cost_value
                    global_best_position = np.copy(
                        particle_position[particle_index])

            iterate = iterate+1
            print("Iteration: ", iterate,
                  " | Global best cost: ", global_fit_value)

        print(global_fit_value)
        print("The best position for each parameter: ",
              global_best_position, " with ", iterate, " iteration.")
        return global_best_position
