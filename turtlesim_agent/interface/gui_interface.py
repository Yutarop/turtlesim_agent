import asyncio

from turtlesim_agent.interface.base_interface import BaseAgentInterface


class GUIAgentInterface(BaseAgentInterface):
    def __init__(self, agent_executor):
        self.agent_executor = agent_executor

    async def send_user_input(self, user_input: str) -> str:
        try:
            result = await self.agent_executor.ainvoke({"input": user_input})
            return result.get("output", "[No response]")
        except Exception as e:
            return f"[Error] {e}"

    def shutdown(self):
        pass  # Any specific shutdown procedures can go here
