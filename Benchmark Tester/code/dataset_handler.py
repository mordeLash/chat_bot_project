import random
from datasets import load_dataset
import ast

class DatasetHandler:
    """
    A handler for loading and preprocessing benchmark datasets 
    such as MMLU, MuSR, SQuAD, and TruthfulQA.
    """

    def __init__(self):
        """
        Initialize dataset loaders. 
        Each key is a dataset name mapped to its respective loader method.
        """
        self.loaders = {
            "MMLU": self._load_mmlu,
            "MUSR": self._load_musr,
            "SQUAD": self._load_squad,
            "TRUTHFUL_QA": self._load_truthfulqa,
            "web_questions": self._load_web_questions,
            "ARC": self._load_ARC,  
        }

    def load(self, dataset_name):
        """
        Load a dataset by name.

        Args:
            dataset_name (str): Name of the dataset (must exist in self.loaders).

        Returns:
            list[dict]: A list of questions with standardized fields.

        Raises:
            ValueError: If dataset_name is unknown.
        """
        if dataset_name not in self.loaders:
            raise ValueError(f"Unknown dataset: {dataset_name}")
        return self.loaders[dataset_name]()

    # ---------- Individual dataset loaders ----------


    def _load_mmlu(self):
        """
        Load the MMLU benchmark dataset.

        Returns:
            list[dict]: Each question contains subject, question, choices, 
                        correct answer text, and answer index.
        """
        subjects = ["human_aging", "marketing", "professional_medicine", "clinical_knowledge", "miscellaneous"]
        questions_list = []
        for subject in subjects:
            dataset = load_dataset("cais/mmlu", subject, split="test")
            for entry in dataset:
                questions_list.append({
                    "subject": subject,
                    "question": entry["question"],
                    "answer": entry["answer"],          # Correct answer index
                    "choices": entry["choices"],        # List of possible answers
                    "answer_index": entry["answer"],    # Same as "answer", kept for consistency
                })
        return questions_list


    def _load_ARC(self):
        """
        Load the ARC benchmark dataset.

        Returns:
            list[dict]: Each question contains subject, question, choices
            and answer index.
        """
        answer_map = {'A': 0, 'B': 1, 'C': 2, 'D': 3,'1': 0, '2': 1, '3': 2, '4': 3}
        questions_list = []
        dataset = load_dataset("allenai/ai2_arc","ARC-Challenge", split="test")
        for entry in dataset:
            questions_list.append({
                "question": entry["question"],
                "choices": entry["choices"]["text"],        # List of possible answers
                "answer_index": answer_map[entry["answerKey"]],    # Correct answer index
            })
        return questions_list



    def _load_musr(self):
        """
        Load the MuSR benchmark dataset.

        Returns:
            list[dict]: Each question contains narrative, question, choices,
                        correct answer index, and the correct choice text.
        """
        subjects = ["object_placements", "team_allocation"]
        questions_list = []
        for subject in subjects:
            dataset = load_dataset("TAUR-Lab/MuSR", split=subject)
            for entry in dataset:
                questions_list.append({
                    "subject": subject,
                    "narrative": entry["narrative"],           # Scenario description
                    "question": entry["question"],             # The actual question
                    "choices": ast.literal_eval(entry["choices"]),  # Convert string list to Python list
                    "answer_index": entry["answer_index"],     # Index of the correct answer
                    "answer_choice": entry["answer_choice"],   # The correct answer text
                })
        return questions_list

    def _load_squad(self):
        """
        Load the SQuAD benchmark dataset (subset by subject titles).

        Returns:
            list[dict]: Each question contains subject, context, question,
                        and a list of correct answers.
        """
        subjects = ["Nikola_Tesla", "Martin_Luther", "Doctor_Who", "Chloroplasts", "Teacher", "Immune_system"]
        questions_list = []
        dataset = load_dataset("squad", split="validation")
        for entry in dataset:
            if entry["title"] in subjects:
                questions_list.append({
                    "subject": entry["title"],
                    "context": entry["context"],                 # Passage to read from
                    "question": entry["question"],               # Open-ended question
                    "correct_answers": entry["answers"]["text"], # All valid correct answers
                })
        return questions_list
    

    def _load_web_questions(self):
        """
        Load the web_questions benchmark dataset 

        Returns:
            list[dict]: Each question contains question
            and a list of correct answers.
        """
        questions_list = []
        dataset = load_dataset("stanfordnlp/web_questions", split="test")
        for entry in dataset:
            questions_list.append({
                "question": entry["question"],               # Open-ended question
                "correct_answers": entry["answers"], # All valid correct answers
                })
        return questions_list

    def _load_truthfulqa(self):
        """
        Load the TruthfulQA dataset.

        Returns:
            list[dict]: Each question contains category, question text,
                        best answer, list of correct answers, list of incorrect answers,
                        and a randomly assigned correct answer index.
        """
        questions_list = []
        dataset = load_dataset("truthfulqa/truthful_qa", "generation", split="validation")
        for entry in dataset:
            formatted = {
                "subject": entry["category"],
                "question": entry["question"],
                "answer": entry["best_answer"],            # The preferred correct answer
                "correct_answers": entry["correct_answers"],
                "incorrect_answers": entry["incorrect_answers"],
            }
            # Randomly pick a correct answer index to align with multiple-choice style benchmarks
            formatted["answer_index"] = random.randint(0, len(entry["incorrect_answers"]))
            questions_list.append(formatted)
        return questions_list
