from llm_app import LLMApp
from colorama import init, Fore, Style



def main():
    """
    Main entry point for the chatbot.
    - Initializes an LLMApp instance with a specified model and token limit.
    - Starts a command-line loop for user input and assistant responses.
    - Exits when the user types "/bye".
    """
    # Create an instance of the chatbot with a small Qwen2 model
    chatBot = LLMApp(model="qwen2:0.5b", num_predict=500,character="signal_proccesing",use_rag=True,)

    # Initial prompt to start conversation
    prompt = input(
        Fore.LIGHTCYAN_EX + "Enter a message to start conversing and /bye to quit\n"
        + Fore.LIGHTWHITE_EX + "\n🧑 User: " + Fore.LIGHTBLUE_EX
    )

    # Conversation loop until user quits
    while prompt != "/bye":
        # Print assistant response header
        print(Fore.LIGHTWHITE_EX + "\n🤖 Assistant: ", end="", flush=True)

        # Generate response from chatbot
        answer = chatBot.chat(prompt,log_to_file=True)

        # Print response in green for readability
        print(Fore.LIGHTGREEN_EX + answer, end="", flush=True)
        print()  # Add a newline after assistant response

        # Prompt for next user input
        prompt = input(Fore.LIGHTWHITE_EX + "\n🧑 User: " + Fore.LIGHTBLUE_EX)


if __name__ == "__main__":
    main()
