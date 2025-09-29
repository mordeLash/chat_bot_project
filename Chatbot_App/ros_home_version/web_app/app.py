import subprocess  # Used to run shell commands and manage processes
from fastapi import FastAPI  # FastAPI framework to create API endpoints
from fastapi.responses import FileResponse  # To serve static files like HTML and CSS
import uvicorn  # ASGI server for running FastAPI apps
import signal  # To handle OS signals (like Ctrl+C)

# Create FastAPI app instance
app = FastAPI()

# Global variable to hold the subprocess running the ROS launch
ssh_process = None

# Template ROS command to launch the audio_transcriber package
# {MODEL_NAME} and {CHARACTER} will be replaced dynamically when the API is called
ROS_COMMAND = "bash -c 'pwd && . devel/setup.bash && roslaunch audio_transcriber audio_transcriber.launch model_name_arg:={MODEL_NAME} character_arg:={CHARACTER}'"


# ----------------------------
# Routes for serving frontend
# ----------------------------

@app.get("/")
def serve_index():
    """
    Serve the main HTML page of the web app.
    """
    return FileResponse("web_app/index.html")


@app.get("/styles.css")
def serve_css():
    """
    Serve the CSS file for styling the web app.
    """
    return FileResponse("web_app/styles.css", media_type="text/css")


# ----------------------------
# Routes for controlling ROS
# ----------------------------

@app.post("/start")
def start_roslaunch(model_name: str, character: str):
    """
    Start the ROS launch process with the given model name and character.

    Args:
        model_name (str): Name of the model to load in ROS.
        character (str): Character parameter for ROS launch.

    Returns:
        dict: Status message indicating that ROS launch has started.
    """
    print(f"{model_name} {character}")
    global ssh_process

    # Only start if no process is currently running
    if ssh_process is None:
        # Replace placeholders with actual values
        ros_command = ROS_COMMAND.replace("{MODEL_NAME}", model_name).replace("{CHARACTER}", character)
        ssh_process = subprocess.Popen(ros_command, shell=True, stdin=subprocess.PIPE, text=True)
        print("Started roslaunch on remote machine")

    return {"status": "roslaunch started"}


@app.post("/stop")
def stop_roslaunch():
    """
    Stop the ROS launch process if it is running and kill related ROS nodes.

    Returns:
        dict: Status message indicating that ROS launch has stopped.
    """
    print("stopping")
    global ssh_process

    if ssh_process is not None:
        # Terminate the subprocess running ROS launch
        ssh_process.terminate()

        # Kill specific ROS nodes that might still be running
        subprocess.run(["rosnode", "kill", "/llm_node", "/asr_transcriber"])

        ssh_process = None
        print("Stopped roslaunch")

    return {"status": "roslaunch stopped"}


# Duplicate stop endpoint for handling OS signals (Ctrl+C, termination)
@app.post("/stop")
def stop_roslaunch_kill(*args):
    """
    Wrapper to stop ROS launch when receiving termination signals.
    """
    stop_roslaunch()


# ----------------------------
# Signal handlers for graceful shutdown
# ----------------------------
signal.signal(signal.SIGINT, stop_roslaunch_kill)   # Handle Ctrl+C
signal.signal(signal.SIGTERM, stop_roslaunch_kill)  # Handle termination signals


# ----------------------------
# Run the FastAPI app
# ----------------------------
if __name__ == "__main__":
    uvicorn.run(app, host="127.0.0.1", port=8000)
