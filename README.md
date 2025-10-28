# Chatbot Engineering and Adaptation Project

[cite_start]This repository contains the final project for a B.Sc. in Engineering from Bar-Ilan University[cite: 13]. [cite_start]The project focuses on the evaluation of Large Language Models (LLMs) and their implementation into a conversational chatbot system, including deployment on a humanoid robot[cite: 37, 38].

The project is divided into two main applications:
1.  [cite_start]**Benchmark Tester**: A flexible system for evaluating and comparing the performance of various LLMs using standard academic benchmarks[cite: 40].
2.  [cite_start]**Chatbot App**: A conversational application that uses the selected LLM, enhanced with a Retrieval-Augmented Generation (RAG) system, and is capable of running on a personal computer or integrated with a robot via ROS[cite: 41, 38].

## Technology Stack

* [cite_start]**LLM Serving:** [Ollama](https://ollama.com/) [cite: 878]
* [cite_start]**LLM Application Framework:** [LangChain](https://python.langchain.com/docs/introduction) [cite: 907]
* [cite_start]**Vector Search & Storage:** [FAISS (Facebook AI Similarity Search)](https://github.com/facebookresearch/faiss) [cite: 943]
* [cite_start]**Robotics Integration:** [ROS (Robot Operating System) Noetic](https://wiki.ros.org/noetic) [cite: 1164]
* **Datasets & Benchmarks:** [Hugging Face Datasets](https://huggingface.co/datasets)
* [cite_start]**Target Robot Platform:** [ARI by PAL Robotics](https://pal-robotics.com/robot/ari) [cite: 53]

---

## 1. Benchmark Tester

[cite_start]A modular and extensible application designed to run LLMs through various benchmarks to evaluate their performance in terms of accuracy, response time, and resource consumption[cite: 40, 46]. [cite_start]The system is built to be flexible, allowing for the easy addition of new models and tests[cite: 48, 413].

### Architecture

[cite_start]The system is designed with a modular structure to separate concerns and improve maintainability[cite: 413, 414].

* [cite_start]`main.py`: The entry point for running tests[cite: 416].
* [cite_start]`test_handler.py`: The core orchestrator that manages the testing workflow, loading configurations, datasets, and models[cite: 421].
* [cite_start]`dataset_handler.py`: Handles loading and preprocessing of datasets from sources like Hugging Face[cite: 439, 440].
* [cite_start]`LLM_handler.py`: A unified interface for creating and interacting with LLMs via Ollama and LangChain[cite: 450].
* [cite_start]`benchmark_handler.py`: Manages the execution of the actual benchmark, sends questions to the LLM, and calculates performance metrics[cite: 456].
* [cite_start]`results_saver.py`: Saves detailed results and summaries to Excel files for analysis[cite: 472, 474].

![Benchmark System Architecture](https://i.imgur.com/5J3C3Qp.png)
[cite_start]*Figure based on the diagram from the project report[cite: 408].*

### Benchmarks Implemented

The tester includes implementations for the following benchmarks:
* [cite_start]**MMLU**: Measures general knowledge across a wide range of subjects[cite: 333, 335].
* [cite_start]**MUSR**: Evaluates multi-step reasoning and analytical thinking capabilities[cite: 343, 345].
* [cite_start]**TruthfulQA**: Tests model reliability and its tendency to avoid generating common falsehoods[cite: 357, 359].
* [cite_start]**SQuAD**: Assesses reading comprehension on a given text[cite: 363].
* [cite_start]**ARC**: Tests knowledge in a wide range of subjects[cite: 371].
* [cite_start]**Web Questions**: Evaluates the model's ability to answer common internet search-style questions[cite: 382, 384].

### Installation and Usage

1.  [cite_start]Navigate to the `Benchmark_Tester` directory[cite: 956].
2.  [cite_start]Install the required Python packages[cite: 957]:
    ```bash
    pip install -r requirements.txt
    ```
3.  [cite_start]Ensure that Ollama is installed and running[cite: 958].
4.  [cite_start]Open `code/main.py` and configure the desired model, parameters, and tests to run[cite: 959].
    * [cite_start]To run a specific test: `testHandler.run_test("test_name")`[cite: 960].
    * [cite_start]To run all tests defined in `config/benchmarks.yaml`: `testHandler.run_all_benchmarks()`[cite: 962].
5.  [cite_start]Run the main script from the `Benchmark_Tester` directory[cite: 964].
    ```bash
    python code/main.py
    ```
6.  [cite_start]Results will be saved as Excel files in the `results/<model_name>/` directory[cite: 963].

### Adding a New Benchmark

[cite_start]The system is designed for easy extension[cite: 504]. [cite_start]To add a new test (e.g., ARC or Web Questions), follow these steps[cite: 968]:
1.  [cite_start]**Find the Dataset**: Locate the desired dataset on Hugging Face[cite: 969].
2.  [cite_start]**Create a Loader**: In `dataset_handler.py`, add a new function to load and parse the dataset into a standardized format[cite: 980].
3.  [cite_start]**Create a Runner**: In `benchmark_handler.py`, add a function to handle the logic of running the specific test type (e.g., multiple-choice or open-ended)[cite: 1063].
4.  [cite_start]**Add Prompts**: Create a new folder in the `prompts` directory with a simple prompt (`_simp.txt`), an engineered prompt (`_eng.txt`), and a template file (`_Prompt_template.txt`)[cite: 1080, 1120].
5.  [cite_start]**Update Config**: Add the new benchmark configuration to the `benchmarks.yaml` file, specifying its name, evaluation method, and prompt paths[cite: 1123, 1124].

---

## 2. Chatbot App

[cite_start]A conversational AI application built using the best-performing model from the evaluation phase (`qwen2.5`)[cite: 693]. [cite_start]The app features different "personas," including an expert on Signal Processing who leverages a RAG system to provide accurate, context-aware answers based on Professor Sharon Ganot's academic papers[cite: 699, 708, 709].

### Architecture

[cite_start]The application can run in a simple command-line mode or as a full-fledged ROS package for integration with the ARI robot[cite: 38]. [cite_start]The core of the application is a RAG pipeline[cite: 710].

![RAG Chatbot Architecture](https://i.imgur.com/vH9v5l2.png)
[cite_start]*Figure based on the diagram from the project report[cite: 757].*

**Workflow:**
1.  [cite_start]A user's message is added to the conversation history[cite: 761].
2.  [cite_start]The last message is converted into a vector embedding[cite: 763].
3.  [cite_start]This embedding is used to search a FAISS vector database for relevant text chunks from the knowledge base (e.g., academic papers)[cite: 764].
4.  [cite_start]The retrieved chunks, along with the full conversation history, are passed as context to the LLM[cite: 765].
5.  [cite_start]The LLM's response is returned to the user and added to the history[cite: 766].

### ROS Integration for ARI Robot

[cite_start]For robotic applications, the chatbot is wrapped in a ROS node[cite: 769]. [cite_start]This system converts spoken language to text, processes it through the RAG pipeline, and sends the generated text back to the robot to be synthesized into speech[cite: 38].

![ROS System Architecture](https://i.imgur.com/e3gT55z.png)
[cite_start]*Figure based on the diagram from the project report[cite: 779].*

1.  [cite_start]The user speaks to the robot; the audio is captured and published to a ROS topic[cite: 781, 782].
2.  [cite_start]The `asr_transcriber` node receives the audio, sends it to Google ASR for transcription, and publishes the resulting text[cite: 782, 785].
3.  [cite_start]The `llm_node` receives the text, sends it to the RAG chat application, and receives the LLM's response[cite: 785, 786, 787].
4.  [cite_start]The response text is published to the robot's Text-to-Speech (TTS) topic to be spoken aloud[cite: 790].

### Installation and Usage

[cite_start]The project provides three versions: a simple terminal app, a home ROS version, and a lab version for the ARI robot[cite: 1260, 1261, 1264].

#### Simple Home Version (No ROS)
1.  [cite_start]Navigate to the `Chatbot_App/home_simple_version/rag_llm` directory[cite: 1206].
2.  [cite_start]Install the required packages from `Chatbot_App/requirements.txt`[cite: 1205].
3.  [cite_start]Ensure Ollama is running at the configured address[cite: 1207].
4.  [cite_start]Select the desired character and model in the `main.py` script and run it[cite: 1208].

#### ROS Home Version
1.  [cite_start]Install ROS Noetic (Ubuntu 20.04 and Python 3.8.10 recommended)[cite: 1160, 1164].
2.  [cite_start]Install required packages from `Chatbot_App/requirements.txt`[cite: 1189].
3.  [cite_start]Navigate to `Chatbot_App/ros_home_version/` and build the ROS workspace[cite: 1191]:
    ```bash
    catkin_make
    source devel/setup.bash
    ```
4.  [cite_start]In a new terminal, start the ROS core: `roscore`[cite: 1192].
5.  [cite_start]Update the Google ASR key path in `asr_transcriber.py` and make the scripts executable (`chmod +x src/audio_transcriber/scripts/*`)[cite: 1193, 1195].
6.  [cite_start]Install a local TTS engine like `espeak-ng`[cite: 1197, 1198].
7.  [cite_start]Ensure Ollama is running[cite: 1199].
8.  [cite_start]Run the web app from the `web_app` directory: `python app.py`[cite: 1200].
9.  [cite_start]Open `http://127.0.0.1:8000/` in your browser and click "Start Command"[cite: 1203].

### Building a New RAG Database

1.  [cite_start]Navigate to the `rag_llm` directory[cite: 1175].
2.  [cite_start]Place your source documents (.pdf, .txt, .md) in a folder[cite: 1177].
3.  [cite_start]In `rag_maker.py`, add a line to process your folder: `db.add_documents_from_folder("path/to/your/docs")`[cite: 1176].
4.  [cite_start]To create a new database from scratch, delete the existing `faiss_index` directory first[cite: 1183].
5.  [cite_start]Run the script to build and save the new FAISS index[cite: 1182]:
    ```bash
    python rag_maker.py
    ```
