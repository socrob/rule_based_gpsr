# Rule-Based Planner

## Overview

The Rule-Based Planner is responsible for generating a sequence of actions that allow a robot to achieve specific goals extracted from user instructions. This planner uses a straightforward, predefined approach: each goal type is associated with a hardcoded sequence of actions that the robot must follow.

## Features
- **Predefined Action Sequences**: Each goal type has a hardcoded sequence of actions associated with it. This ensures predictable and consistent plan generation for goals within the predefined scope.

- **Deterministic Planning**: The planner always generates the same sequence of actions for each specific goal type, making it reliable in environments where tasks are repetitive or well-structured.

- **Simple Failure Handling**: If an action fails, the planner immediately halts execution and marks the plan as unsuccessful. There is no retry or error recovery mechanism, keeping the logic simple but limiting the planner’s robustness in dynamic or failure-prone environments.

- **Efficient Execution**: Since the plans are predefined and hardcoded, the planner generates and executes them quickly without needing complex decision-making or computational resources.

- **Limited Flexibility**: The planner is not designed to adapt to new or unexpected goals or tasks outside of its hardcoded sequences. It is best suited for structured, static environments with predictable tasks.

## Requirements

- ROS version: Noetic
- Dependencies:
  - [socrob_speech_msgs](https://github.com/socrob/socrob_speech_msgs) 
  - [socrob_planning_msgs](https://github.com/socrob/socrob_planning_msgs)

## Installation

### 0. Install the message modules
Follow the installation instructions in the [socrob_speech_msgs](https://github.com/socrob/socrob_speech_msgs) and [socrob_planning_msgs](https://github.com/socrob/socrob_planning_msgs) repositories.

### 1. Clone the repository
```bash
cd ~/<your_workspace>/src
git clone https://github.com/socrob/rule_based_gpsr.git
```

### 2. Build the workspace
Navigate to your catkin workspace and build the package:

```bash
cd ~/<your_workspace>
catkin build
```

### 3. Source the setup file
After building, source the workspace to update the environment:
```bash
source ~/<your_workspace>/devel/setup.bash
```

# Usage
## Launching the Node
To launch the node, use the following command:

```bash
roslaunch rule_based_gpsr simple_rule_based_planner.launch
```

This will launch the planner and the NLU modules.

## Launch file arguments
- `nlu_transcript_topic` - The topic where the NLU is expecting the transcript to be published.
- `nlu_result_topic` - The topic where the NLU will publish the result.
- `planner_name` - The name given to the planner node. The planner node will expect to receive instruction in the `<planner_name>/instruction` as a `std_msgs/String` and will publish the resulting actions in the `<planner_name>/actions` topic as `socrob_planning_msgs/action_msg`.
- `old_GPSR`, `old_EGPSR` and `new_GPSR` - Variables to choose which NLU to use. 

# Available Actions

## Object Interaction Messages

- `find_object` - Receives an object name (e.g., pringles, coke, sponge), category (e.g., snacks, drinks, cleaning supplies), or description (e.g., smallest object, heaviest drink), and optionally an associated location. It attempts to detect an object matching the description at the specified location or at a default location if none is provided. Upon success, it returns the object's ID from the semantic map.
- `find_objects` - Similar to `find_object`, but continues searching beyond the first match. For example, if searching for snacks in the kitchen, it returns IDs of all snacks found in the kitchen.
- `describe_object` - Receives an object's ID and generates a description of that object, including specified properties (e.g., object name, category).
- `describe_objects` - Creates a description for multiple objects provided as input.
- `pick` - Receives the ID of an object from the semantic map, moves to its location, and attempts to pick it up. If the pick fails, it requests a nearby human to assist by placing the object on the robot's gripper.
- `place` - Receives the name of a surface and attempts to place the object currently held by the robot onto it. If unsuccessful, it requests assistance from a nearby person to place the object correctly.

## People Interaction Messages

- `find_person` - Receives identifying features of a person (e.g., name, pose, age, gender, clothes), and optionally a location. It searches for the person in the specified location or across all rooms if no location is provided. It returns an ID used to identify the person.
- `find_people` - Similar to `find_person`, but continues searching beyond the first match, returning IDs of all people matching the description.
- `describe_person` - Receives a person's ID and generates a description of that person, including requested properties (e.g., name, age range, pose).
- `guide` - Guides a person to a specified location, periodically confirming that the person, identified by their ID, is following the robot.
- `follow` - Follows a person, identified by their ID, to a specified location or until the person requests the robot to stop.
- `ask_question` - Receives a question as a string, asks the person the question, and returns the response.
- `speak` - Uses a text-to-speech engine to vocalize a specified text.
- `answer` - Receives a question (e.g., "what time is it?" or "tell me a joke") and retrieves the appropriate response from a database to communicate to the person.
- `deliver` - Delivers an object previously picked up to a person. This action assumes that the robot is currently interacting with someone, so it must follow a `find_person` action.

## Generic Actions

- `count` - Receives a string that represents a list of items separated by commas and counts them. This action can be used with `find_people` or `find_objects` to count the number of people or objects in a room that match a certain description.
- `move` - Moves the robot to a desired location.