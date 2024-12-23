## Control of a Nonlinear Liquid-Level System
This project investigates the control of a nonlinear two-tank liquid-level system under external disturbances using reinforcement learning (RL). The aim is to compare two RL approaches—value approximation and policy approximation (neural networks)—and evaluate their performance in optimizing system stability, energy efficiency, and precision under challenging conditions. The project incorporates the design of an adaptive reward function to enhance robustness and performance.

## Key features include:
###
* **Reinforcement Learning Framework**: Utilized value and policy approximation to design control strategies for the two-tank system.
* **Adaptive Reward Structure**: Balanced steady-state accuracy, energy efficiency, and disturbance mitigation.  
* **Disturbance Handling**: Tested controllers under random noise, sinusoidal disturbances, and step changes to assess resilience.  
* **Simulation and Evaluation**: Compared original and adaptive reward functions for RL-based controllers.  

## Results:
###
* The Value Approximation method minimized control effort but struggled with steady-state precision..
* The Policy Approximation approach achieved better accuracy and smoother control actions, particularly under adaptive rewards.  
* Adaptive rewards improved overall robustness, showing promise for future work in handling uncertainties. 

## Contributors
* Ratnesh Yadav
* Parastou Fahim 
