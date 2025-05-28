from abc import ABC, abstractmethod


class BaseAgentInterface(ABC):
    @abstractmethod
    def send_user_input(self, user_input: str) -> str:
        """Send user input to the agent and return the response."""

    @abstractmethod
    def shutdown(self):
        """Perform cleanup operations when shutting down."""
