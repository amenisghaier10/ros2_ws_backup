# AI-Driven Robot – Emotion Detection Node

This project implements a **ROS 2 node (`emotion_node`)** that enables an intelligent robot to recognize and respond to human emotions in real time. The node integrates **facial expression recognition (CNN)** and **text-based emotion analysis (Transformer)**, then communicates results via WebSocket for visualization or robot control.  

A simple frontend can be launched to display detected emotions and responses.  

---

## Prerequisites  

Make sure the following are installed:  

ROS 2 (Humble or Rolling) → [Install ROS 2](https://docs.ros.org/en/humble/Installation.html)  
Python 3.8+  
pip (Python package manager)  

If ROS 2 is not installed, this project will not work. Please follow the official ROS 2 installation guide for your OS.  

---

## Setup Instructions  

1. Source ROS 2 environment:  
```bash
source /opt/ros/rolling/setup.bash
```  

2. Build the project:  
```bash
colcon build
```  

3. Source your workspace:  
```bash
source install/setup.bash
```  

---

## Python Dependencies  

Upgrade pip and install required packages:  
```bash
python3 -m pip install --upgrade pip
pip3 install torch torchvision transformers opencv-python websockets
```  

---

## Running the Application  

- **Terminal 1:** Run the ROS 2 node  
```bash
ros2 run emotion_detector emotion_node
```  

- **Terminal 2:** Launch the frontend interface  
```bash
python3 -m http.server 8765
```  

Open in your browser:  
[http://localhost:8765](http://localhost:8765)  

---

## Notes  

- The node integrates:  
  - **CNN model** for facial expression recognition  
  - **Transformer model** (`j-hartmann/emotion-english-distilroberta-base`) for text-based emotion analysis  
- All detected emotions are published as JSON messages over WebSocket (`port 8765`).  
- Responses are **predefined for each detected emotion**, but can be extended with reinforcement learning for adaptive interactions.  

---

## Example Workflow  

You enter:  
```
I'm feeling really great today!
```  

The system detects:  
```
Emotion: joy (0.99 confidence)
```

The robot (or frontend) responds:  
```
"I'm glad to hear that! What made your day so good?"
```  

Similarly, if a webcam is connected, the node can classify facial expressions (e.g., happy, sad, angry) and publish the result in the same unified format.  

---

## Future Enhancements  

- Integration with robot actuators (gestures, movement, speech).  
- Reinforcement learning for adaptive responses.  
- Multimodal recognition (voice, gestures).  
