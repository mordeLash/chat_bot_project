
# Project Title

A brief description of what this project does and who it's for

# Chatbot Engineering and Adaptation Project

This repository contains the final project for a B.Sc. in Engineering from Bar-Ilan University. The project focuses on the evaluation of Large Language Models (LLMs) and their implementation into a conversational chatbot system, including deployment on a humanoid robot.

The project is divided into two main applications:
1.  **Benchmark Tester**: A flexible system for evaluating and comparing the performance of various LLMs using standard academic benchmarks.
2.  **Chatbot App**: A conversational application that uses the selected LLM, enhanced with a Retrieval-Augmented Generation (RAG) system, and is capable of running on a personal computer or integrated with a robot via ROS.

## Technology Stack

* **LLM Serving:** [Ollama](https://ollama.com/) 
* **LLM Application Framework:** [LangChain](https://python.langchain.com/docs/introduction)
* **Vector Search & Storage:** [FAISS (Facebook AI Similarity Search)](https://github.com/facebookresearch/faiss)
* **Robotics Integration:** [ROS (Robot Operating System) Noetic](https://wiki.ros.org/noetic)
* **Datasets & Benchmarks:** [Hugging Face Datasets](https://huggingface.co/datasets)
* **Target Robot Platform:** [ARI by PAL Robotics](https://pal-robotics.com/robot/ari)

---

## 1. Benchmark Tester

A modular and extensible application designed to run LLMs through various benchmarks to evaluate their performance in terms of accuracy, response time, and resource consumption. 
The system is built to be flexible, allowing for the easy addition of new models and tests.

### Architecture

![code structure](chat_bot_project/assets/Benchmarking_diagram.png)

The system is designed with a modular structure to separate concerns and improve maintainability.

* `main.py`: The entry point for running tests.
* `test_handler.py`: The core orchestrator that manages the testing workflow, loading configurations, datasets, and models.
* `dataset_handler.py`: Handles loading and preprocessing of datasets from sources like Hugging Face.
* `LLM_handler.py`: A unified interface for creating and interacting with LLMs via Ollama and LangChain.
* `benchmark_handler.py`: Manages the execution of the actual benchmark, sends questions to the LLM, and calculates performance metrics.
* `results_saver.py`: Saves detailed results and summaries to Excel files for analysis.


### Benchmarks Implemented

The tester includes implementations for the following benchmarks:
* **MMLU**: Measures general knowledge across a wide range of subjects.
* **MUSR**: Evaluates multi-step reasoning and analytical thinking capabilities.
* **TruthfulQA**: Tests model reliability and its tendency to avoid generating common falsehoods.
* **SQuAD**: Assesses reading comprehension on a given text.
* **ARC**: Tests knowledge in a wide range of subjects.
* **Web Questions**: Evaluates the model's ability to answer common internet search-style questions.


our results:

![weighted sum graph](chat_bot_project/assets/Weighted_acc_graph.png)

### Installation and Usage

1.  Navigate to the `Benchmark_Tester` directory.
2.  Install the required Python packages:
    ```bash
    pip install -r requirements.txt
    ```
3.  Ensure that Ollama is installed and running.
4.  Open `code/main.py` and configure the desired model, parameters, and tests to run.
    * To run a specific test: `testHandler.run_test("test_name")`.
    * To run all tests defined in `config/benchmarks.yaml`: `testHandler.run_all_benchmarks()`.
5.  Run the main script from the `Benchmark_Tester` directory.
    ```bash
    python code/main.py
    ```
6.  Results will be saved as Excel files in the `results/<model_name>/` directory.

### Adding a New Benchmark

The system is designed for easy extension. To add a new test (e.g., ARC or Web Questions), follow these steps:
1.  **Find the Dataset**: Locate the desired dataset on Hugging Face.
2.  **Create a Loader**: In `dataset_handler.py`, add a new function to load and parse the dataset into a standardized format.
3.  **Create a Runner**: In `benchmark_handler.py`, add a function to handle the logic of running the specific test type (e.g., multiple-choice or open-ended).
4.  **Add Prompts**: Create a new folder in the `prompts` directory with a simple prompt (`_simp.txt`), an engineered prompt (`_eng.txt`), and a template file (`_Prompt_template.txt`).
5.  **Update Config**: Add the new benchmark configuration to the `benchmarks.yaml` file, specifying its name, evaluation method, and prompt paths.

---

## 2. Chatbot App

A conversational AI application built using the best-performing model from the evaluation phase (`qwen2.5`). The app features different "personas," including an expert on Signal Processing who leverages a RAG system to provide accurate, context-aware answers based on Professor Sharon Ganot's academic papers.

### Architecture

The application can run in a simple command-line mode or as a full-fledged ROS package for integration with the ARI robot. The core of the application is a RAG pipeline.

![Chatbot Architecture](chat_bot_project/assets/Chatbot_diagram.png)


**Workflow:**
1.  A user's message is added to the conversation history.
2.  The last message is converted into a vector embedding.
3.  This embedding is used to search a FAISS vector database for relevant text chunks from the knowledge base (e.g., academic papers).
4.  The retrieved chunks, along with the full conversation history, are passed as context to the LLM.
5.  The LLM's response is returned to the user and added to the history.


### Examples

![Joke expert example](chatbot_git/chat_bot_project/assets/Joke_conversation_example.png)

![Signal Processing expert example](chatbot_git/chat_bot_project/assets/Signal_conversation_example.png)


### ROS Integration for ARI Robot

For robotic applications, the chatbot is wrapped in a ROS node. This system converts spoken language to text, processes it through the RAG pipeline, and sends the generated text back to the robot to be synthesized into speech.

![ROS App Architecture](chat_bot_project/assets/ROS_diagram.png)

1.  The user speaks to the robot; the audio is captured and published to a ROS topic.
2.  The `asr_transcriber` node receives the audio, sends it to Google ASR for transcription, and publishes the resulting text.
3.  The `llm_node` receives the text, sends it to the RAG chat application, and receives the LLM's response.
4.  The response text is published to the robot's Text-to-Speech (TTS) topic to be spoken aloud.

### Installation and Usage

The project provides three versions: a simple terminal app, a home ROS version, and a lab version for the ARI robot.

#### Simple Home Version (No ROS)
1.  Navigate to the `Chatbot_App/home_simple_version/rag_llm` directory.
2.  Install the required packages from `Chatbot_App/requirements.txt`.
3.  Ensure Ollama is running at the configured address.
4.  Select the desired character and model in the `main.py` script and run it.

#### ROS Home Version
1.  Install ROS Noetic (Ubuntu 20.04 and Python 3.8.10 recommended.
2.  Install required packages from `Chatbot_App/requirements.txt`.
3.  Navigate to `Chatbot_App/ros_home_version/` and build the ROS workspace:
    ```bash
    catkin_make
    source devel/setup.bash
    ```
4.  In a new terminal, start the ROS core: `roscore`.
5.  Update the Google ASR key path in `asr_transcriber.py` and make the scripts executable (`chmod +x src/audio_transcriber/scripts/*`).
6.  Install a local TTS engine like `espeak-ng`.
7.  Ensure Ollama is running.
8.  Run the web app from the `web_app` directory: `python app.py`.
9.  Open `http://127.0.0.1:8000/` in your browser and click "Start Command".

### Building a New RAG Database

1.  Navigate to the `rag_llm` directory.
2.  Place your source documents (.pdf, .txt, .md) in a folder.
3.  In `rag_maker.py`, add a line to process your folder: `db.add_documents_from_folder("path/to/your/docs")`.
4.  To create a new database from scratch, delete the existing `faiss_index` directory first.
5.  Run the script to build and save the new FAISS index:
    ```bash
    python rag_maker.py
    ```
