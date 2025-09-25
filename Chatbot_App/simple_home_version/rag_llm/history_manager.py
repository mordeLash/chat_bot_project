from langchain_core.messages import SystemMessage, HumanMessage, AIMessage


class HistoryManager:
    """
    Manages the conversation history between system, user, and assistant.
    Stores messages as LangChain BaseMessage objects.
    """

    def __init__(self):
        # List of SystemMessage, HumanMessage, AIMessage
        self.chat_history = []

    def getHistory(self):
        """Return the full message history as a List[BaseMessage]."""
        return self.chat_history

    def addSystemPrompt(self, message: str):
        """Add a system-level instruction message."""
        self.chat_history.append(SystemMessage(content=message))

    def addAiMessage(self, message: str):
        """Add an assistant (AI) response message."""
        self.chat_history.append(AIMessage(content=message))

    def addHumanMessage(self, message: str):
        """Add a human (user) message."""
        self.chat_history.append(HumanMessage(content=message))

    def getRecentUserContext(self, n: int = 1) -> str:
        """
        Return the last `n` user+assistant turns as text (for retrieval).
        Each turn is formatted as:
            User: ...
            Assistant: ...
        """
        rendered = self.getHistory()
        turns = []
        i = len(rendered) - 1

        while i >= 1 and len(turns) < n:
            if isinstance(rendered[i], AIMessage) and isinstance(rendered[i - 1], HumanMessage):
                turn = f"User: {rendered[i - 1].content}\nAssistant: {rendered[i].content}"
                turns.insert(0, turn)  # insert at front to preserve order
                i -= 2
            else:
                i -= 1

        return "\n\n".join(turns)
