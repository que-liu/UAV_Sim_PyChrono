import random
import numpy as np
import matplotlib.pyplot as plt
from deap import base, creator, tools, algorithms

# --- Mock Simulation Error Function ---
def simulate_tracking_error(Kp_x):
    """
    Fake simulation: assume there's an ideal Kp that gives minimum error.
    This function just mimics a U-shaped error curve.
    """
    ideal_Kp = 2.5
    noise = random.gauss(0, 0.1)
    error = (Kp_x - ideal_Kp) ** 2 + noise  # Minimum at Kp_x = 2.5
    return error

# --- DEAP Setup ---
creator.create("FitnessMin", base.Fitness, weights=(-1.0,))  # Minimize error
creator.create("Individual", list, fitness=creator.FitnessMin)

toolbox = base.Toolbox()
toolbox.register("attr_float", random.uniform, 0.0, 5.0)  # Kp_x range
toolbox.register("individual", tools.initRepeat, creator.Individual, toolbox.attr_float, n=1)
toolbox.register("population", tools.initRepeat, list, toolbox.individual)

def evaluate(individual):
    Kp_x = individual[0]
    error = simulate_tracking_error(Kp_x)
    return (error,)  # DEAP expects a tuple

toolbox.register("evaluate", evaluate)
toolbox.register("mate", tools.cxBlend, alpha=0.5)
toolbox.register("mutate", tools.mutGaussian, mu=0, sigma=0.2, indpb=1.0)
toolbox.register("select", tools.selTournament, tournsize=3)

# --- Run GA ---
population = toolbox.population(n=20)
NGEN = 30
CXPB, MUTPB = 0.7, 0.3

best_errors = []
for gen in range(NGEN):
    offspring = algorithms.varAnd(population, toolbox, cxpb=CXPB, mutpb=MUTPB)
    
    fits = list(map(toolbox.evaluate, offspring))
    for fit, ind in zip(fits, offspring):
        ind.fitness.values = fit
    
    population = toolbox.select(offspring, k=len(population))
    top_ind = tools.selBest(population, k=1)[0]
    best_errors.append(top_ind.fitness.values[0])

    print(f"Gen {gen:02d} | Best Kp_x: {top_ind[0]:.3f} | Error: {top_ind.fitness.values[0]:.4f}")

# --- Plot Error over Generations ---
plt.plot(best_errors)
plt.title("Best Tracking Error per Generation")
plt.xlabel("Generation")
plt.ylabel("Tracking Error")
plt.grid(True)
plt.show()
