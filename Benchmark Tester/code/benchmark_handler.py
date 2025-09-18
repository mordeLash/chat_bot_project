import re
from tqdm import tqdm

class BenchmarkHandler:
    """
    A handler class for running and evaluating different LLM benchmarks
    such as MMLU, MuSR, TruthfulQA, and SQuAD. 
    Provides utilities for running multiple-choice and open-ended benchmarks, 
    formatting inputs, evaluating outputs, and calculating performance metrics.
    """

    def __init__(self, chatbot):
        """
        Initialize the BenchmarkHandler.

        Args:
            chatbot: An object with an `invoke` method for querying the model.
        """
        self.chatbot = chatbot
        self.benchmarks = {
            "MMLU": self._run_mmlu,
            "MUSR": self._run_musr,
            "TRUTHFUL_QA": self._run_truthful_qa,
            "TRUTHFUL_QA_OPEN": self._run_truthful_qa_open,
            "SQUAD": self._run_squad,
            "web_questions": self._run_web_questions,
            "ARC": self._run_ARC,
        }

    def run(self, benchmark_name, data):
        """
        Run a given benchmark.

        Args:
            benchmark_name (str): The benchmark identifier (e.g., "MMLU").
            data (list): Benchmark dataset.

        Returns:
            list: Model responses formatted as dictionaries.
        """
        if benchmark_name not in self.benchmarks:
            raise ValueError(f"Unknown benchmark: {benchmark_name}")
        return self.benchmarks[benchmark_name](data)

    # ---------- Benchmark Runners ----------

    def _run_mmlu(self, data):
        """
        Run the MMLU multiple-choice benchmark.
        """
        return self._run_multiple_choice(
            data, 
            lambda q, c: {"question": q["question"], "choices": c, "subject": q["subject"]}
        )
    
    def _run_ARC(self, data):
        """
        Run the ARC multiple-choice benchmark.
        """
        return self._run_multiple_choice(
            data, 
            lambda q, c: {"question": q["question"], "choices": c}
        )
    def _run_web_questions(self, data):
        """
        Run the web_questions open-ended benchmark.
        """
        responses = []
        for question in tqdm(data):
            response = self.chatbot.invoke({"question": question["question"]})
            responses.append(self._obj_to_dict(response))
        return responses

    def _run_musr(self, data):
        """
        Run the MuSR multiple-choice benchmark.
        """
        return self._run_multiple_choice(
            data,
            lambda q, c: {"question": q["question"], "narrative": q["narrative"], "choices": c}
        )

    def _run_truthful_qa(self, data):
        """
        Run the TruthfulQA benchmark in multiple-choice mode.
        Builds the choice set dynamically since incorrect answers are provided separately.
        """
        def formatter(q, choices_text):
            return {"question": q["question"], "choices": choices_text}
        
        def choice_builder(q):
            # Construct choices by mixing incorrect answers with the correct one
            choices = q['incorrect_answers'][:]
            if q['answer_index'] < len(q['incorrect_answers']):
                choices.append(q['incorrect_answers'][q['answer_index']])
                choices[q['answer_index']] = q['answer']
            else:
                choices.append(q['answer'])
            return choices
        
        return self._run_multiple_choice(data, formatter, custom_choice_builder=choice_builder)

    def _run_truthful_qa_open(self, data):
        """
        Run the TruthfulQA benchmark in open-ended mode (no multiple-choice).
        """
        responses = []
        for question in tqdm(data):
            response = self.chatbot.invoke({"question": question["question"]})
            responses.append(self._obj_to_dict(response))
        return responses

    def _run_squad(self, data):
        """
        Run the SQuAD benchmark (open-ended, requires question + context).
        """
        responses = []
        for question in tqdm(data):
            response = self.chatbot.invoke({"question": question["question"], "context": question["context"]})
            responses.append(self._obj_to_dict(response))
        return responses

    # ---------- Shared Helpers ----------

    def _run_multiple_choice(self, data, formatter, custom_choice_builder=None):
        """
        Generic runner for multiple-choice style benchmarks.

        Args:
            data (list): Benchmark dataset.
            formatter (function): Function to format input for the chatbot.
            custom_choice_builder (function, optional): Function to build choices.

        Returns:
            list: Responses converted into dictionaries.
        """
        responses = []
        for question in tqdm(data):
            choices = (
                custom_choice_builder(question) 
                if custom_choice_builder else question["choices"]
            )
            choices_text = self._format_choices(choices)
            response = self.chatbot.invoke(formatter(question, choices_text))
            responses.append(self._obj_to_dict(response))
        return responses

    def _format_choices(self, choices):
        """
        Convert a list of choices into a formatted string.
        Example:
            0) choice A
            1) choice B
        """
        return "".join(f"{i}) {c}\n" for i, c in enumerate(choices))

    @staticmethod
    def _obj_to_dict(obj):
        """
        Convert an object to a dictionary, excluding private attributes.
        """
        return {k: v for k, v in vars(obj).items() if not k.startswith("_")}

    # ---------- Evaluations ----------

    def eval_multiple_choice(self, data, responses):
        """
        Evaluate multiple-choice responses by matching the model's answer
        (formatted as [index]) against the correct answer index.

        Returns:
            float: Accuracy percentage.
        """
        correct_answers = 0.0
        for response, question in zip(responses, data):
            response_index = "bad formatting"
            # Look for answers in the form [n]
            letter_match = re.search(r"\[(\d)\]", response["content"])
            response["answer"] = question["answer_index"]
            response["correct"] = "NO"
            if letter_match:
                response_index = int(letter_match.group(1))
                if response_index == question["answer_index"]:
                    correct_answers += 1
                    response["correct"] = "YES"
        accuracy = (correct_answers / len(responses)) * 100
        return accuracy

    def slm_as_referee(self, data, responses, incorrect_answers=False):
        """
        Use a secondary LLM (referee) to judge correctness of responses.

        Args:
            data (list): Benchmark dataset with ground truth answers.
            responses (list): Model responses.
            incorrect_answers (bool): Whether the referee gets refrence incorrect answers.

        Returns:
            float: Accuracy percentage.
        """
        correct_answers = 0.0
        for response, question in tqdm(zip(responses, data), total=len(responses)):
            payload = {
                "question": question["question"],
                "response": response["content"],
                "correct_answers": question["correct_answers"],
            }
            if incorrect_answers:
                payload["incorrect_answers"] = question["incorrect_answers"]

            ref_response = self.referee.invoke(payload)
            response["answer"] = question["correct_answers"]
            response["correct"] = ref_response.content
            if ref_response.content[:3].lower() == "yes":
                correct_answers += 1
        accuracy = (correct_answers / len(responses)) * 100
        return accuracy

    def set_referee(self, referee):
        """
        Set a referee chatbot for evaluation.
        """
        self.referee = referee
    
    def calc_times(self, responses):
        """
        Aggregate token usage and timing statistics from model responses.

        Args:
            responses (list): List of response dictionaries with metadata.

        Returns:
            dict: Contains totals, throughput, and latency metrics.
        """
        totals = {
            "total_tokens": 0,
            "input_tokens": 0,
            "output_tokens": 0,
            "total_duration": 0,
            "load_duration": 0,
            "eval_duration": 0,
            "prompt_eval_duration": 0,
        }
        for r in responses:
            totals["total_tokens"] += r["usage_metadata"]["total_tokens"]
            totals["input_tokens"] += r["usage_metadata"]["input_tokens"]
            totals["output_tokens"] += r["usage_metadata"]["output_tokens"]
            totals["total_duration"] += r["response_metadata"]["total_duration"]
            totals["load_duration"] += r["response_metadata"]["load_duration"]
            totals["prompt_eval_duration"] += r["response_metadata"]["prompt_eval_duration"]
            totals["eval_duration"] += r["response_metadata"]["eval_duration"]

        sec = 1e-9  # Convert nanoseconds to seconds
        tokens_per_second = totals["total_tokens"] / (totals["total_duration"] * sec)
        output_tokens_per_second = totals["output_tokens"] / (totals["total_duration"] * sec)
        time_per_question = (totals["total_duration"] * sec) / len(responses)

        return {
            **{k: v * sec if "duration" in k else v for k, v in totals.items()},
            "tokens_per_second": tokens_per_second,
            "output_tokens_per_second": output_tokens_per_second,
            "time_per_question": time_per_question,
        }
