import json
import time
from openai import OpenAI

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from tiago_pick_service.srv import PickPose

# Configuration
OPENAI_API_KEY = "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"
OPENAI_COMPLETION_MODEL = "gpt-4o"  # Options: "gpt-4o", "gpt-4o-mini", "gpt-3.5-turbo"
#OPENAI_COMPLETION_MODEL = "gpt-4o-mini"
# OPENAI_COMPLETION_MODEL = "gpt-3.5-turbo"

def call_pick_service(input_name, x, y, z, roll, pitch, yaw):
    rclpy.init()
    node = rclpy.create_node('pick_client')

    client = node.create_client(PickPose, 'pick_pose')
    while not client.wait_for_service(timeout_sec=2.0):
        print("Waiting for service...")

    request = PickPose.Request()
    request.object_name = input_name
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    pose.orientation.w = 1.0
    request.pose = pose

    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    result = future.result()
    rclpy.shutdown()

    return {"status": "success" if result.success else "failed"}

def generate_text(prompt, max_length=512):
    """Generate text using the selected OpenAI model, print raw response."""
    client = OpenAI(api_key=OPENAI_API_KEY)
    response = client.chat.completions.create(
        model=OPENAI_COMPLETION_MODEL,
        messages=[
            {"role": "system", "content": "You are Holland, a patient care robot (RHA) in a hospital. Interpret clinician commands, select tasks, respond in plain English."},
            {"role": "user", "content": prompt}
        ],
        max_tokens=max_length,
        temperature=0.7,
        top_p=0.9
    )
    raw_response = response.choices[0].message.content.strip()
    #print(f"[RAW LLM RESPONSE]: {raw_response}")
    return raw_response

# Empty tool implementations
def Navigate(input_name, x, y, z, roll, pitch, yaw):
    print(f"Initiating Navigate to {input_name} at x:{x}, y:{y}, z:{z}, roll:{roll}, pitch:{pitch}, yaw:{yaw}")
    time.sleep(2)
    print(f"Navigate to {input_name} ongoing...")
    time.sleep(2)
    print(f"Navigate to {input_name} completed")
    time.sleep(2)
    return {"status": "success"}

def Pick(input_name, x, y, z, roll, pitch, yaw):
    print(f"Initiating Pick {input_name} at x:{x}, y:{y}, z:{z}, roll:{roll}, pitch:{pitch}, yaw:{yaw}")
    time.sleep(2)
    print(f"Pick {input_name} ongoing...")
    result = call_pick_service(input_name, x, y, z, roll, pitch, yaw)
    time.sleep(2)
    print(f"Pick {input_name} completed")
    time.sleep(2)
    # return {"status": "success"}
    return result
    # return call_pick_service(input_name, x, y, z, roll, pitch, yaw)

def Place(input_name, x, y, z, roll, pitch, yaw):
    print(f"Initiating Place object at {input_name} with x:{x}, y:{y}, z:{z}, roll:{roll}, pitch:{pitch}, yaw:{yaw}")
    time.sleep(2)
    print(f"Place object at {input_name} ongoing...")
    time.sleep(2)
    print(f"Place object at {input_name} completed")
    time.sleep(2)
    return {"status": "success"}

def Tool_5(input_name):
    print(f"Initiating generic action Tool 5 on {input_name}")
    time.sleep(2)
    print(f"Generic action Tool 5 on {input_name} ongoing...")
    time.sleep(2)
    print(f"Generic action Tool 5 on {input_name} completed")
    time.sleep(2)
    return {"status": "success"}

def Tool_6(input_name):
    print(f"Initiating generic action Tool 6 at {input_name}")
    time.sleep(2)
    print(f"Generic action Tool 6 at {input_name} ongoing...")
    time.sleep(2)
    print(f"Generic action Tool 6 at {input_name} completed")
    time.sleep(2)
    return {"status": "success"}

class MultiStepAgent:
    def __init__(self):
        self.memory = [{"role": "system", "content": "You are Holland, a patient care robot (RHA) in a hospital."}]  # Only identity
    
    def run(self, user_task):
        self.memory.append({"role": "user", "content": user_task})
        
        # Select task and get actions
        task, action_tools = self.select_task()
        print(f"[task, action_tools]: {task, action_tools}")
        self.memory.append({"role": "task", "content": task})
        
        # Fetch coordinates and arguments for actions
        actions_with_args = self.fetch_coordinates(action_tools)
        print(f"[actions_with_args]: {actions_with_args}")
        self.memory.append({"role": "action", "content": json.dumps(actions_with_args)})
        
        # Execute actions and get task status
        task_status = self.execute_actions(task, actions_with_args)
        self.memory.append({"role": "assistant", "content": f"Task {task} completed: {task_status}"})

        return self.summarize()
        
    def select_task(self):
        task_specs = [
            {
                "name": "Fetch Medication",
                "description": "Bring medication",
                "actions": [
                    {"name": "Navigate to Medication", "tool": "Navigate"},
                    {"name": "Pick Medication", "tool": "Pick"},
                    {"name": "Navigate to Patient Room", "tool": "Navigate"},
                    {"name": "Place Medication", "tool": "Place"}
                ]
            },
            {
                "name": "Task 2",
                "description": "Task 2",
                "actions": [
                    {"name": "Action 5", "tool": "Tool 5"},
                    {"name": "Action 6", "tool": "Tool 6"}
                ]
            }
        ]
        prompt = f"""Memory: {json.dumps(self.memory)}
        Task specifications: {json.dumps(task_specs, indent=2)}
        Select a task based on the user query and task specifications.
        Return JSON: {{"task": "task_name", "actions": [{{"action": "action1", "tool": "tool1"}}, {{"action": "action2", "tool": "tool2"}}, ...]}}
        """
        response = json.loads(generate_text(prompt))
        return response["task"], response["actions"]
    
    def fetch_coordinates(self, action_tools):
        
        prompt = f"""
        Memory: {json.dumps(self.memory)}
        
        Tool signatures and descriptions: 
        - Navigate (input_name, x, y, z, roll, pitch, yaw): Moves the robot to a specified location or object, receiving a location or object name and its coordinates.
        - Pick (input_name, x, y, z, roll, pitch, yaw): Picks up a specified object, receiving the object name and its coordinates.
        - Place (input_name, x, y, z, roll, pitch, yaw): Places the held object at a specified location, receiving the location name (not object name) and its coordinates.
        - Tool_5 (input_name): Performs a generic action on a specified object, receiving the object name.
        - Tool_6 (input_name): Performs a generic action at a specified location, receiving the location name.
        
        Coordinates: {{'medication 1': {{'x': 1, 'y': 2, 'z': 3, 'roll': 0, 'pitch': 0, 'yaw': 0}}, 'medication 2': {{'x': 10, 'y': 22, 'z': 3, 'roll': 0, 'pitch': 0, 'yaw': 0}}, 'patient room 1': {{'x': 5, 'y': 5, 'z': 1, 'roll': 0, 'pitch': 0, 'yaw': 0}}, 'patient room 2': {{'x': 15, 'y': 15, 'z': 1, 'roll': 0, 'pitch': 0, 'yaw': 0}}}}.
        
        For action-tools {json.dumps(action_tools)}, assign arguments based on the user query and action context:
        - Infer input_name from the user query and action name, selecting from the Coordinates dictionary (e.g., 'medication 1' for medication actions, 'patient room 1' for location actions).
        - For Navigate, Pick, and Place, include x, y, z, roll, pitch, yaw from the Coordinates dictionary matching the input_name.
        - For Tool 5 and Tool 6, include only the input_name.
        Return only the JSON object, without any text, explanations, or markdown:
        [{{"tool": "tool_name", "input_name": "name", "x": 1, ...}}, ...]
        """
        # Get raw response
        raw_response = generate_text(prompt)
        
        # Safeguard: Extract JSON from response
        try:
            # Remove markdown or text, extract JSON
            json_start = raw_response.find('[')
            json_end = raw_response.rfind(']') + 1
            if json_start != -1 and json_end != -1:
                json_str = raw_response[json_start:json_end]
            else:
                json_str = raw_response
            return json.loads(json_str)
        except json.JSONDecodeError as e:
            print(f"[ERROR] Failed to parse JSON: {e}, Raw Response: {raw_response}")
            # Fallback: Return empty list or retry (adjust as needed)
            return []
    
    def execute_actions(self, task, actions_with_args):
        """Execute actions for the task and return task status."""
        task_status = "success"
        for action in actions_with_args:
            print(f"Action {action['tool']} initiated for {action['input_name']}")
            tool_result = self.execute_tool(action)
            if tool_result["status"] != "success":
                task_status = "failed"
                print(f"Action {action['tool']} failed")
                break
            print(f"Action {action['tool']} completed")
        return task_status
    
    def execute_tool(self, action):
        try:
            if action["tool"] == "Navigate":
                return Navigate(
                    action["input_name"],
                    action.get("x", 0),
                    action.get("y", 0),
                    action.get("z", 0),
                    action.get("roll", 0),
                    action.get("pitch", 0),
                    action.get("yaw", 0)
                )
            elif action["tool"] == "Pick":
                return Pick(
                    action["input_name"],
                    action.get("x", 0),
                    action.get("y", 0),
                    action.get("z", 0),
                    action.get("roll", 0),
                    action.get("pitch", 0),
                    action.get("yaw", 0)
                )
            elif action["tool"] == "Place":
                return Place(
                    action["input_name"],
                    action.get("x", 0),
                    action.get("y", 0),
                    action.get("z", 0),
                    action.get("roll", 0),
                    action.get("pitch", 0),
                    action.get("yaw", 0)
                )
            elif action["tool"] == "Tool 5":
                return Tool_5(action["input_name"])
            elif action["tool"] == "Tool 6":
                return Tool_6(action["input_name"])
            else:
                print(f"[ERROR] Unknown tool: {action['tool']}")
                return {"status": "failed"}
        except Exception as e:
            print(f"[ERROR] Tool {action['tool']} execution failed: {e}")
            return {"status": "failed"}
    
    def summarize(self):
        prompt = f"""Memory: {json.dumps(self.memory)}
        Summarize the completed task, including its status (success or failed).
        Ask: 'What else would you like me to do?'
        Return strictly the plain text summary, with no markdown or additional formatting."""
        return generate_text(prompt)

def demo():
    print("="*50)
    print("HOLLAND MULTI-STEP AGENT DEMO")
    print("="*50)
    user_task = input("Enter task: ") or "Bring medication 2 to patient room 2"
    print("\nProcessing...\n")
    agent = MultiStepAgent()
    result = agent.run(user_task)
    print("OUTPUT:")
    print("-"*50)
    print(result)
    print("-"*50)

if __name__ == "__main__":
    demo()