# ROS Turtlesim: Batman's Ninja Turtles Battle Royale

A multiplayer turtle battle game built with ROS1 and Turtlesim, where each player controls a turtle using predefined keys for movement and attacks. The system includes a game engine to manage state, control turtle actions, and handle combat, with the goal of being the last turtle standing.

## ✨ Key Features

* **Multiplayer Control:** Allows multiple turtles to be controlled simultaneously using keyboard inputs.
* **Game Engine Node:** Manages the overall game state, tracking the health and attack counts of each turtle.
* **Combat System:** Turtles can attack others within a specified radius, inflicting damage and reducing their own available attacks.
* **Health Management:** Each turtle starts with 100 health points and is removed from the game if its health drops to 0.
* **Game Over Logic:** The game ends when all turtles have exhausted their 10 attacks, and the winner is the turtle with the highest remaining health.
