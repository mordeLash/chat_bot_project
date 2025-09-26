import dataset_handler
import LLM_handler
import benchmark_handler
import results_saver
import os
import datetime
import yaml

class BenchmarkRunner:
    def __init__(self, 
                 model="qwen0.5b", 
                 temp=0.0, 
                 path="results/", 
                 max_len=1, 
                 num_threads=-1, 
                 num_gpu=-1, 
                 referee="llama3.1", 
                 config_file="config/benchmarks.yaml"):
        """
        BenchmarkRunner handles loading datasets, running benchmarks,
        evaluating model responses, and saving results.

        Args:
            model (str): LLM model name to benchmark.
            temp (float): Temperature for response randomness.
            path (str): Directory to save benchmark results.
            max_len (int): Max number of dataset samples to use.
            num_threads (int): Threads used by the model (-1 = default).
            num_gpu (int): GPUs used by the model (-1 = default).
            referee (str): Model to act as referee for evaluation.
            config_file (str): YAML file with benchmark definitions.
        """
        self.model = model
        self.temp = temp
        self.path = path
        self.max_len = max_len
        self.num_threads = num_threads
        self.num_gpu = num_gpu
        self.referee = referee

        # Load all benchmark configurations from YAML
        with open(config_file, "r") as f:
            self.benchmark_configs = yaml.safe_load(f)

    def _get_llm_vars(self):
        """
        Build a dictionary of LLM parameters for ChatOllama.
        Includes temperature, threads, and GPU configuration.
        """
        LLM_vars = {"temperature": self.temp}
        LLM_vars = {"num_predict": 128}
        if self.num_threads != -1:
            LLM_vars["num_thread"] = self.num_threads
        if self.num_gpu != -1:
            LLM_vars["num_gpu"] = self.num_gpu
        return LLM_vars

    def _get_prompts(self, SP, PT):
        """
        Load system and prompt template files from the prompts directory.

        Args:
            SP (str): System prompt filename (without extension).
            PT (str): Prompt template filename (without extension).

        Returns:
            (system_prompt, prompt_template): Tuple of Tuples.
        """
        system_prompt, prompt_template = None, None
        if SP and SP.lower() != "none":
            system_text = self._read_file(f"prompts/{SP}.txt")
            system_prompt = ("system", system_text)
        if PT and PT.lower() != "none":
            prompt_text = self._read_file(f"prompts/{PT}.txt")
            prompt_template = ("user", prompt_text)
        return system_prompt, prompt_template
    
    def _read_file(self, file_path):
        """
        Utility function to read a text file and return its contents.
        """
        with open(file_path, 'r') as file:
            lines = file.read()
        return lines

    def run_test(self, test="MMLU"):
        """
        Run a single benchmark test end-to-end:
        - Load dataset
        - Query model
        - Evaluate responses
        - Save results
        """
        if test not in self.benchmark_configs:
            print(f"Invalid benchmark name: {test}")
            return

        # Load benchmark config for this test
        cfg = self.benchmark_configs[test]
        benchmark = test

        # === File setup ===
        os.makedirs(self.path, exist_ok=True)  # Ensure results directory exists
        timestamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")

        # Get prompt filenames (SP/PT) from config
        SP = cfg.get("SP", "none")
        PT = cfg.get("PT", "none")

        # Define output file paths
        summary_file = os.path.join(
            self.path,
            f"{benchmark}_{self.model}_{timestamp}_summary.xlsx"
        )
        responses_file = os.path.join(
            self.path,
            f"{benchmark}_{self.model}_{timestamp}_responses.xlsx"
        )

        # === LLM setup ===
        LLM_vars = self._get_llm_vars()
        system_prompt, prompt_template = self._get_prompts(SP, PT)
        chatbot = LLM_handler.getChatBot(self.model, system_prompt, prompt_template, LLM_vars)

        # Initialize benchmark and dataset handlers
        benchmarkHandler = benchmark_handler.BenchmarkHandler(chatbot)
        datasetHandler = dataset_handler.DatasetHandler()

        # === Dataset ===
        dataset = datasetHandler.load(cfg["dataset_name"])
        dataset = dataset[:self.max_len]  # Limit dataset size

        # === Run benchmark ===
        print(f"starting benchmark: {benchmark} on model: {self.model}")
        responses = benchmarkHandler.run(cfg["dataset_name"], dataset)

        # === Evaluation ===
        acc = None
        if cfg["eval"] == "multiple_choice":
            # Direct accuracy evaluation
            acc = benchmarkHandler.eval_multiple_choice(dataset, responses)

        elif cfg["eval"] == "referee":
            # Use a referee model to evaluate answers
            ref = self.referee if self.referee != "none" else self.model

            # Load referee prompts from config
            ref_SP = cfg.get("referee_SP", "none")
            ref_PT = cfg.get("referee_PT", "none")
            referee_system, referee_template = self._get_prompts(ref_SP, ref_PT)

            # Create referee chatbot
            referee_chatbot = LLM_handler.getChatBot(ref, referee_system, referee_template)
            benchmarkHandler.set_referee(referee=referee_chatbot)

            # Evaluate with referee model
            acc = benchmarkHandler.slm_as_referee(
                data=dataset,
                responses=responses,
                incorrect_answers=cfg.get("incorrect_answers", False)
            )

        # === Save results ===
        # Save raw model responses
        dict_responses = [{"content": r["content"]} for r in responses]
        results_saver.save_objects_to_excel(dict_responses, responses_file)

        # Save benchmark summary (metadata + accuracy)
        test_res = benchmarkHandler.calc_times(responses)
        test_res.update({
            "system_prompt": str(system_prompt),
            "prompt_template": str(prompt_template),
            "benchmark": benchmark,
            "model": self.model,
            "SP": SP,
            "PT": PT,
            "temp": self.temp,
            "num_questions": len(responses),
            "accuracy": f"{acc}%",
        })
        results_saver.save_objects_to_excel([test_res], summary_file)

        print(f"The accuracy is {acc}%")

    def run_all_benchmarks(self):
        """
        Run every benchmark defined in the config file sequentially.
        """
        for b in self.benchmark_configs.keys():
            self.run_test(b)
