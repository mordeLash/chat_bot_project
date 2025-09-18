from langchain_ollama import ChatOllama
from langchain_core.prompts import ChatPromptTemplate

def getChatBot(model_name, system_prompt, prompt_template, LLM_vars={'temperature': 0.0}):
    """
    Create and configure a chatbot using LangChain + Ollama.

    Args:
        model_name (str): The name of the Ollama model to use (e.g., "llama2").
        system_prompt (tuple): A system-level message definition for ChatPromptTemplate,
                               usually in the form ("system", "You are a helpful assistant.").
        prompt_template (tuple): The user/content message template,
                                 e.g. ("user", "Answer the question: {question}").
        LLM_vars (dict, optional): Extra parameters passed to ChatOllama, such as:
            - temperature (float): Controls randomness of outputs (default: 0.2).
            - top_p, top_k, max_tokens, etc. depending on Ollama support.

    Returns:
        RunnableSequence: A LangChain chain object that formats prompts and 
                          passes them to the Ollama model.
    """
    # Initialize Ollama-backed chatbot with provided model and parameters
    chatBot = ChatOllama(
        model=model_name,
        **LLM_vars,      # Pass in additional config (temperature, top_p, etc.)
    )

    # Build a chat-style prompt template (system + user prompt)
    prompt = ChatPromptTemplate.from_messages([system_prompt, prompt_template])

    # Create a chain: prompt formatting → model call
    chain = prompt | chatBot

    return chain
