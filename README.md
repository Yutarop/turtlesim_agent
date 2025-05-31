![turtlesim_agent_logo](https://github.com/user-attachments/assets/282f0708-41b6-45f9-9ce4-c749014e9183)
![ROS2-humble Industrial CI](https://github.com/Yutarop/turtlesim_agent/actions/workflows/ros2_ci.yml/badge.svg)
## Project Overview

`turtlesim_agent` is an AI agent that transforms the classic ROS [turtlesim](http://wiki.ros.org/turtlesim) simulator into a creative canvas driven by natural language. Powered by LangChain, this AI agent interprets text-based instructions and translates them into visual drawings, turning the simulated turtle into a digital artist. This project explores how large language models can interact with external environments to exhibit creative behavior. Users can describe shapes or drawing intentions in plain English, and the AI agent reasons through the instructions to execute them using turtlesim's motion commands.


## 🌈 TurtleSim Agent Demo: Drawing a Rainbow
#### Prompt Used
> I want you to draw a rainbow composed of 7 semi-circular arcs, each with a different color and a radius ranging from 2.0 cm to 2.7 cm. The colors should follow the traditional rainbow order: violet, indigo, blue, green, yellow, orange, red with the pen's width of 5. Please offset the starting position of each semi-circle by 0.1 cm to avoid overlap.

https://github.com/user-attachments/assets/ea9aee14-bdff-4f2b-8b49-c262b09e9051

Note: This demo was generated using the `gemini-2.0-flash` model.
Please note that results may vary even when using the same model, due to the non-deterministic nature of language models. Outputs may differ depending on factors like prompt phrasing, timing, or model updates.


## 🛠 Getting Started
#### Requirements
- ROS 2 Humble Hawksbill (This project has only been tested with ROS 2 Humble. Compatibility with other ROS 2 distributions is not guaranteed.)
- Python 3.10+
- Other dependencies as listed in `requirements.txt`
### 1. Clone and build in a ROS2 workspace 
```bash
$ cd ~/{ROS_WORKSPACE}/src
$ git clone -b humble-devel https://github.com/Yutarop/turtlesim_agent.git
$ python3 -m pip install -r turtlesim_agent/requirements.txt
$ cd ~/{ROS_WORKSPACE} && colcon build
```
### 2. API Key Setup

`turtlesim_agent` supports multiple language model providers via LangChain. You need to set API keys for the providers you intend to use.

#### 🔐 Add API Keys to Your Shell Configuration

To make your API keys available in your development environment, add them to your shell configuration file (e.g., `~/.bashrc`, `~/.zshrc`), then reload the file using `source`.

```bash
export OPENAI_API_KEY=your_openai_api_key
export ANTHROPIC_API_KEY=your_anthropic_api_key
export GOOGLE_API_KEY=your_google_api_key
export COHERE_API_KEY=your_cohere_api_key
export MISTRAL_API_KEY=your_mistral_api_key
```
   > 💡 You only need to set the keys for the providers you plan to use. 

#### 🖥️ Using Self-Hosted LLMs (e.g., Ollama)
If you're running a local or remote LLM server (e.g., via Ollama), specify the server URL as follows:
```bash
export URL="https:your_ollama_server_ip:11434"
```
### 3. (Optional) Enable Tracing with LangSmith
To trace and debug agent behavior using LangSmith, set the following environment variables:

Basic Tracing Configuration:
```bash
export LANGSMITH_ENDPOINT="https://api.smith.langchain.com"
export LANGSMITH_TRACING=false
```
Full Configuration with API Key and Project Name:
```bash
export LANGSMITH_ENDPOINT="https://api.smith.langchain.com"
export LANGSMITH_TRACING=true
export LANGSMITH_API_KEY=your_api_key_here
export LANGSMITH_PROJECT=your_project_name_here
```

### 4. Set LLM Models
To specify which Large Language Model (LLM) your agent should use, you need to configure the model name in two places:

- `turtlesim_node.py`
- `turtlesim_agent.launch.xml` (only if you use ROS 2 launch files)
#### **Edit the default model name**

   In both `turtlesim_node.py` and `turtlesim_agent.launch.xml`, update the `agent_model` parameter to match the model you want to use.

   - **Python ([turtlesim_node.py](https://github.com/Yutarop/turtlesim_agent/blob/main/turtlesim_agent/turtlesim_node.py)):**

     ```python
     self.declare_parameter("agent_model", "gemini-2.0-flash")
     ```

   - **Launch file ([turtlesim_agent.launch.xml](https://github.com/Yutarop/turtlesim_agent/blob/main/launch/turtlesim_agent.launch.xml)):**

     ```xml
     <arg name="agent_model" default="gemini-2.0-flash"/>
     ```

   > 💡 The default model is `"gemini-2.0-flash"`. Replace it with your preferred model name (e.g., `"gpt-4"`, `"claude-3-opus"`, etc.).

#### **Ensure LangChain supports your model**

   If you specify a custom model name, make sure it is supported by LangChain. You can verify this by checking or updating the logic inside `llm.py`.

   - If the model is not yet handled, add a corresponding case in [`llm.py`](https://github.com/Yutarop/turtlesim_agent/blob/main/turtlesim_agent/llms.py) to load the model correctly.
   - Refer to [LangChain documentation](https://docs.langchain.com/docs/) for the latest supported models and providers.


### 5. Apply the changes
Once you have configured the variables, proceed to build and apply the changes to finalize the setup:
```bash
$ cd ~/{ROS_WORKSPACE} && colcon build
$ source ~/.bashrc  # or source ~/.zshrc
```
## ▶️ How to Run
`turtlesim_agent` offers two modes of interaction:  
- A **CLI-based interface**, recommended for debugging and understanding the agent’s internal reasoning.  
- A **GUI-based chat interface**, ideal for intuitive and user-friendly interaction.

#### ⌨️ Run with CLI (Recommended for Development)
```bash
$ ros2 run turtlesim turtlesim_node
$ ros2 run turtlesim_agent turtlesim_agent_node
```
#### 🖼️ Run with GUI (Chat Interface)
```bash
$ ros2 launch turtlesim_agent turtlesim_agent.launch.xml
```

## 🧰 Provided Tools for the AI Agent
`turtlesim_agent` utilizes the tools implemented in the `tools/` directory as callable functions that it can invoke during the reasoning process to accomplish user-defined drawing tasks. 

#### 📁 File Structure
```
tools/
├── all_tools.py # Imports and exports all available tools for the agent
├── math_tools.py # Basic arithmetic and geometric calculations
├── status_tools.py # Queries the current status of the turtle (e.g., position, orientation)
├── motion_tools.py # Controls the movement of the turtle (e.g., forward, rotate)
├── pen_tools.py # Manages pen states (e.g., color, on/off, width)
└── simulation_tools.py # Resets simulation or spawns new turtles
```
#### 🚀 Extending the Agent's Creativity
One of the core ideas behind this project is **enabling creative expression through tool augmentation**. If you'd like to enhance the agent's capabilities further, feel free to add your own tools to the `tools/` directory.

To make new tools available:
1. Create a new `*_tools.py` file in the `tools/` directory.
2. Define your custom functions using LangChain-compatible signatures.
3. Import them in `all_tools.py` so that the agent can access them.

## 🧪 Experiment

To evaluate the drawing capabilities of the `turtlesim_agent`, we defined 10 levels of shape complexity. The table below presents how each LLM agent performs shape drawing via natural language prompts in turtlesim. 

| Level | Shape                            | gpt-4o-mini | gemini-2.0-flash | mistral | claude-3-opus |
|-------|----------------------------------|-------------|------------------|---------|----------------|
| 1     | Circle                           | —           | —                | —       | —              |
| 1     | Square                           | —           | —                | —       | —              |
| 1     | Triangle                         | —           | —                | —       | —              |
| 2     | Rectangle                        | —           | —                | —       | —              |
| 2     | Parallelogram                    | —           | —                | —       | —              |
| 2     | Pentagon                         | —           | —                | —       | —              |
| 3     | Ellipse                          | —           | —                | —       | —              |
| 3     | Star shape made of lines        | [⭕](images/gpt-4o-mini/level3_star_shape_made_of_lines.png)           | —                | —       | —              |
| 3     | Circle inside a triangle         | —           | —                | —       | —              |
| 4     | Hexagon                          | —           | —                | —       | —              |
| 4     | Octagon                          | —           | —                | —       | —              |
| 4     | Decagon                          | —           | —                | —       | —              |
| 5     | Circle inside a triangle         | —           | —                | —       | —              |
| 5     | Triangle inside a square         | —           | —                | —       | —              |
| 5     | Square inside a circle           | —           | —                | —       | —              |
| 6     | Cube (2D representation)         | —           | —                | —       | —              |
| 6     | Cylinder (2D view)               | —           | —                | —       | —              |
| 6     | Cone (2D view)                   | —           | —                | —       | —              |
| 7     | Star-shaped polygon              | —           | —                | —       | —              |
| 7     | Overlapping circles              | —           | —                | —       | —              |
| 7     | Combined triangles               | —           | —                | —       | —              |
| 8     | Spiral                           | —           | —                | —       | —              |
| 8     | Wave shape                       | —           | —                | —       | —              |
| 8     | Part of a fractal                | —           | —                | —       | —              |
| 9     | Irregular polygon                | —           | —                | —       | —              |
| 9     | Complex curve combination        | —           | —                | —       | —              |
| 9     | Geometric pattern                | —           | —                | —       | —              |
| 10    | Torus (2D view)                  | —           | —                | —       | —              |
| 10    | Mandelbrot fractal section       | —           | —                | —       | —              |
| 10    | Non-Euclidean geometric shape    | —           | —                | —       | —              |


> ⭕ = Successfully drawn  
> ❌ = Failed or incorrect output  
> —  = Not yet tested

## 🤝 Contributing
We welcome any ideas that make TurtleSim Agent more creative and expressive.
Whether it's new tools, smarter models, better prompts, experimental results, or entirely new use cases — feel free to open an issue or pull request.
