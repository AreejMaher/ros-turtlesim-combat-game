# ROS Turtlesim: Batman's Ninja Turtles Battle Royale

A multiplayer turtle battle game built with ROS1 and Turtlesim, where each player controls a turtle using predefined keys for movement and attacks. The system includes a game engine to manage state, control turtle actions, and handle combat, with the goal of being the last turtle standing.

## ✨ Key Features

* **Multiplayer Control:** Allows multiple turtles to be controlled simultaneously using keyboard inputs.
* **Game Engine Node:** Manages the overall game state, tracking the health and attack counts of each turtle.
* **Combat System:** Turtles can attack others within a specified radius during a 1-second attack duration, inflicting 50 damage per attack.
* **Health Management:** Each turtle starts with 100 health points and is removed from the game if its health drops to 0 or below.
* **Limited Attacks:** Each turtle has exactly 10 attacks available throughout the game.
* **Game Over Logic:** The game ends when all turtles have exhausted their 10 attacks, and the winner is the turtle with the highest remaining health.

## 🚀 How to Run the Game

### 1. Prerequisites & Setup

* Make sure you have a working ROS1 installation.
* Clone this repository into the `src` folder of your Catkin workspace.
* Build the workspace: `catkin_make`
* Source the workspace: `source devel/setup.bash`
* Install the `getch` library for Python: `pip install getch`

### 2. Network Configuration (Multi-Device Setup)

If you are running the game across multiple devices, you must configure the ROS network properly.

#### Step 1: Find IP Addresses
On each device, find its IP address using:
```bash
ifconfig
```

Example IP addresses:
- Device 1 (Master): 192.168.1.4
- Device 2 (Slave 1): 192.168.1.5
- Device 3 (Slave 2): 192.168.1.6
- Device 4 (Slave 3): 192.168.1.7
- Device 5 (Slave 4): 192.168.1.8

#### Step 2: Configure ROS Network
Edit the bashrc file on each device:
```bash
nano ~/.bashrc
```

**On Master Device (Device 1):**
Add these lines to ~/.bashrc:
```bash
export ROS_MASTER_URI=http://192.168.1.4:11311  # Master device IP
export ROS_IP=192.168.1.4                       # Master device IP
```

**On Slave Device 1 (Device 2):**
Add these lines to ~/.bashrc:
```bash
export ROS_MASTER_URI=http://192.168.1.4:11311  # Master device IP
export ROS_IP=192.168.1.5                       # This device's IP
```

**On Slave Device 2 (Device 3):**
Add these lines to ~/.bashrc:
```bash
export ROS_MASTER_URI=http://192.168.1.4:11311  # Master device IP
export ROS_IP=192.168.1.6                       # This device's IP
```

**On Slave Device 3 (Device 4):**
Add these lines to ~/.bashrc:
```bash
export ROS_MASTER_URI=http://192.168.1.4:11311  # Master device IP
export ROS_IP=192.168.1.7                       # This device's IP
```

**On Slave Device 4 (Device 5):**
Add these lines to ~/.bashrc:
```bash
export ROS_MASTER_URI=http://192.168.1.4:11311  # Master device IP
export ROS_IP=192.168.1.8                       # This device's IP
```

After editing, reload the bashrc file on each device:
```bash
source ~/.bashrc
```

**Important:** Ensure all devices are connected to the same Wi-Fi network and can ping each other.

### 3. Launch the Core System (Master Terminal)

In your first terminal, run these commands in order to start the core game services:

```bash
# Start the ROS Master
roscore
```

```bash
# In a new terminal, start the Turtlesim simulator
rosrun turtlesim turtlesim_node
```

```bash
# In another new terminal, spawn the initial turtles
rosrun my_turtle_fight spawn_turtles.py
```
```bash
# In another new terminal, start the health management node
rosrun my_turtle_fight health_node.py
```

### 4. Launch Player 1 Controls (Player 1 Terminal)

For the first player, run the following commands. Each command should be run in a separate terminal.

```bash
# Terminal 1: Keyboard input for Turtle 1
rosrun my_turtle_fight keyboard.py __name:=keyboard1 turtle:=turtle1
```

```bash
# Terminal 2: Movement controller for Turtle 1
rosrun my_turtle_fight turtle_movement.py __name:=turtle1_movement turtle:=turtle1
```

```bash
# Terminal 3: Attack controller for Turtle 1
rosrun my_turtle_fight attack_node.py __name:=attack_node1 turtle:=turtle1
```

### 5. Launch Player 2 Controls (Slave Device 1)

For the second player, run the following commands. Each command should be run in a separate terminal.

```bash
# Terminal 1: Keyboard input for Turtle 2
rosrun my_turtle_fight keyboard.py __name:=keyboard2 turtle:=turtle2
```

```bash
# Terminal 2: Movement controller for Turtle 2
rosrun my_turtle_fight turtle_movement.py __name:=turtle2_movement turtle:=turtle2
```

```bash
# Terminal 3: Attack controller for Turtle 2
rosrun my_turtle_fight attack_node.py __name:=attack_node2 turtle:=turtle2
```

### 6. Launch Player 3 Controls (Slave Device 2)

For the third player, run the following commands. Each command should be run in a separate terminal.

```bash
# Terminal 1: Keyboard input for Turtle 3
rosrun my_turtle_fight keyboard.py __name:=keyboard3 turtle:=turtle3
```

```bash
# Terminal 2: Movement controller for Turtle 3
rosrun my_turtle_fight turtle_movement.py __name:=turtle3_movement turtle:=turtle3
```

```bash
# Terminal 3: Attack controller for Turtle 3
rosrun my_turtle_fight attack_node.py __name:=attack_node3 turtle:=turtle3
```

### 7. Launch Player 4 Controls (Slave Device 3)

For the fourth player, run the following commands. Each command should be run in a separate terminal.

```bash
# Terminal 1: Keyboard input for Turtle 4
rosrun my_turtle_fight keyboard.py __name:=keyboard4 turtle:=turtle4
```

```bash
# Terminal 2: Movement controller for Turtle 4
rosrun my_turtle_fight turtle_movement.py __name:=turtle4_movement turtle:=turtle4
```

```bash
# Terminal 3: Attack controller for Turtle 4
rosrun my_turtle_fight attack_node.py __name:=attack_node4 turtle:=turtle4
```

## 🎮 Game Rules & Controls

### Movement Controls
Each turtle can be controlled using the following predefined keys:
- **W** - Move up
- **A** - Move left  
- **S** - Move down
- **D** - Move right

### Attack System
- **Q** - Attack key
- Each attack has a **1-second duration**
- Damages other turtles within a **specified radius** during the attack
- Each attack inflicts **50 damage** to targets within range
- Each turtle has a **limited supply of 10 attacks** total
- Each attack reduces the attacking turtle's available attacks by 1

### Health System
- Each turtle starts with **100 health points**
- If a turtle's health drops to **0 or below**, it is **removed from the game**
- Health is reduced by 50 points per attack received

### Victory Conditions
The game ends when **all turtles have exhausted their 10 attacks**. The winner is determined by the turtle with the **highest remaining health**.

## 🏗️ System Architecture

### Game Engine Node
- Manages the overall game state
- Tracks health and attack counts for each turtle
- Detects collisions between turtles during attack duration
- Handles damage calculations and applies damage to targets within attack radius
- Logs scores and determines the winner when the game ends
- Monitors game termination condition (all attacks exhausted)

### Turtle Controller Nodes
- Controls individual turtle movements and actions
- Processes predefined key inputs (W,A,S,D for movement, Q for attack)
- Enables turtles to move in any direction using movement keys
- Handles attack execution with 1-second duration and radius-based damage
- Enforces the 10-attack limit per turtle

**Master Device:** Runs the core game systems and displays the game window
- ROS Master (roscore)
- Turtlesim simulator
- Turtle spawning system
- Health management node

**Slave Devices:** Each controls one turtle (up to 4 slave devices)
- Keyboard input handler
- Movement controller
- Attack controller