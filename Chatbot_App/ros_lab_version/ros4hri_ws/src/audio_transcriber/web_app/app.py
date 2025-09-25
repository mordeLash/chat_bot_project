import subprocess
from fastapi import FastAPI
from fastapi.responses import FileResponse
import uvicorn
import signal
app = FastAPI()
ssh_process = None



REMOTE_IP = "127.0.0.1"
REMOTE_USER = "project"
ROS_COMMAND = "docker exec -i nostalgic_brahmagupta bash -c 'cd /home/ros/git/chat_bot_project/Chatbot_App/ros_lab_version/ros4hri_ws && pwd && . devel/setup.bash && roslaunch audio_transcriber audio_transcriber.launch model_name_arg:={MODEL_NAME} character_arg:={CHARACTER}'"

@app.get("/")
def serve_index():
    return FileResponse("web_app/index.html")


@app.get("/styles.css")
def serve_css():
    return FileResponse("web_app/styles.css", media_type="text/css")


@app.post("/start")
def start_roslaunch(model_name:str,character:str):
    print(f"{model_name} {character}")
    global ssh_process
    if ssh_process is None:
        ros_command=ROS_COMMAND.replace("{MODEL_NAME}",model_name).replace("{CHARACTER}",character)
        full_command = f'ssh {REMOTE_USER}@{REMOTE_IP} "{ros_command}"'
        print(full_command)
        ssh_process = subprocess.Popen(full_command, shell=True, stdin=subprocess.PIPE,text=True)
        print("Started roslaunch on remote machine")
    return {"status": "roslaunch started"}

@app.post("/stop")
def stop_roslaunch():
    print("stopping")
    global ssh_process
    if ssh_process is not None:
        ssh_process.terminate()
        subprocess.run(["rosnode", "kill" ,"/llm_node" ,"/asr_transcriber"])
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

if __name__ == "__main__":
    uvicorn.run(app, host="127.0.0.1", port=8000)
