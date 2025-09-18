import time
from datetime import datetime
import re
import os
from pathlib import Path

from rag_database import RAGDatabase
from history_manager import HistoryManager

from langchain_core.output_parsers import StrOutputParser
from langchain_ollama import ChatOllama
from langchain_core.messages import HumanMessage, SystemMessage
from colorama import init

# Enable colored terminal output (autoreset so colors don't leak)
init(autoreset=True)


def count_tokens(text: str) -> int:
    """
    Roughly count the number of tokens in a string.
    Splits text into words or symbols.
    """
    return len(re.findall(r"\w+|\S", text))


class LLMApp:
    """
    LLM-based application with optional Retrieval-Augmented Generation (RAG) support.
    Maintains conversation history and can log interactions for analysis.
    """

    def __init__(self, model: str = "qwen:0.5b", num_predict: int = 500,
                 character: str = "joke_expert", use_rag: bool = False):
        # Core components
        self.DB = RAGDatabase()
        self.history = HistoryManager()
        self.use_rag = use_rag

        # LLM model setup
        self.model = ChatOllama(
            model=model,
            base_url="http://132.70.226.188:11435/",
            num_predict=num_predict,
        )

        # Base path for local resources
        base_path = Path(__file__).resolve().parent

        # Character system prompt file (relative to rag_llm/character_prompts/)
        self.systemPromptFile = base_path / "character_prompts" / character

        # Load system prompt or fallback to default
        if self.systemPromptFile.exists():
            with open(self.systemPromptFile, "r", encoding="utf-8") as f:
                self.systemPrompt = f.read()
        else:
            self.systemPrompt = (
                "You are a helpful AI Assistant named Ari.\n"
                "Be concise—your answers are never longer than 5 sentences, "
                "unless explicitly asked for a long answer.\n"
                "Always aim to provide accurate, focused, and trustworthy responses."
            )

        self.history.addSystemPrompt(self.systemPrompt)

        # Log file (relative to rag_llm/)
        self.log_file = base_path / "chat_log.txt"

    def build_prompt(self, rag_chunks: str, user_prompt: str):
        """
        Construct the prompt messages for the LLM,
        combining system/history messages, optional RAG context, and user input.
        """
        messages = self.history.getHistory().copy()

        if rag_chunks:
            context_block = f"--- Retrieved Context ---\n{rag_chunks}"
            messages.append(SystemMessage(content=context_block))

        messages.append(HumanMessage(content=user_prompt))
        return messages

    def chat(self, user_prompt: str, log_to_file: bool = False) -> str:
        """
        Handle a new user prompt:
        - Retrieves context (if RAG enabled)
        - Builds prompt
        - Sends request to the LLM
        - Updates history
        - Optionally logs interaction
        """
        if not user_prompt:
            return ""

        start_time = time.time()
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

        # Build retrieval query from most recent context + user prompt
        recent_context = self.history.getRecentUserContext(n=1)
        retrieval_query = f"{recent_context}\nUser: {user_prompt}" if recent_context else user_prompt

        # Retrieve context with RAG (if enabled)
        rag_chunks = self.DB.queryDB(retrieval_query, log_to_file=log_to_file) if self.use_rag else ""
        rag_token_count = count_tokens(rag_chunks)

        # Build final prompt
        prompt_messages = self.build_prompt(rag_chunks, user_prompt)

        # Count tokens in the full prompt
        full_prompt_text = "\n".join([m.content for m in prompt_messages])
        total_tokens = count_tokens(full_prompt_text)

        # Get response (non-streaming for now)
        chain = self.model | StrOutputParser()
        full_response = chain.invoke(prompt_messages)

        # Update history
        self.history.addHumanMessage(user_prompt)
        self.history.addAiMessage(full_response)

        elapsed_time = time.time() - start_time

        # Save log if enabled
        if log_to_file:
            with open(self.log_file, "a", encoding="utf-8") as f:
                f.write(f"\n\n[Chat @ {timestamp}]\n")
                f.write(f"User Prompt:\n{user_prompt}\n")
                f.write(f"Recent Context:\n{recent_context}\n")
                f.write(f"Retrieval Query:\n{retrieval_query}\n")
                f.write(f"RAG Chunks:\n{rag_chunks}\n")
                f.write(f"RAG Token Count: {rag_token_count}\n")
                f.write(f"Prompt Token Count: {total_tokens}\n")
                f.write(f"LLM Response:\n{full_response}\n")
                f.write(f"Total Response Time: {elapsed_time:.2f} seconds\n")

        return full_response
