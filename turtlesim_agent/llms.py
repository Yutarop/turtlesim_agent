import os

from langchain.agents import create_react_agent, create_tool_calling_agent
from langchain.hub import pull
from langchain_anthropic import ChatAnthropic
from langchain_google_genai import ChatGoogleGenerativeAI
from langchain_ollama import ChatOllama
from langchain_openai import ChatOpenAI

from turtlesim_agent.prompts import prompt

URL = os.getenv("URL")


def create_agent(model_name: str, tools: list, temperature: float):
    if model_name == "gemini-2.0-flash":
        llm = ChatGoogleGenerativeAI(model="gemini-2.0-flash", temperature=temperature)
        return create_tool_calling_agent(llm, tools=tools, prompt=prompt)
    elif model_name == "claude":
        llm = ChatAnthropic(model="claude-3-opus-20240229", temperature=temperature)
        return create_tool_calling_agent(llm, tools=tools, prompt=prompt)
    elif model_name == "gpt-4o-mini":
        llm = ChatOpenAI(model="gpt-4o-mini", temperature=temperature)
        return create_tool_calling_agent(llm, tools=tools, prompt=prompt)
    elif model_name == "mistral":
        llm = ChatOllama(model="mistral", base_url=URL, temperature=temperature)
        try:
            return create_react_agent(llm, tools=tools, prompt=prompt)
        except Exception:
            react_prompt = pull("hwchase17/react")
            return create_react_agent(llm, tools=tools, prompt=react_prompt)
    elif model_name == "gemma3":
        llm = ChatOllama(model="gemma3", base_url=URL, temperature=temperature)
        try:
            return create_react_agent(llm, tools=tools, prompt=prompt)
        except Exception:
            react_prompt = pull("hwchase17/react")
            return create_react_agent(llm, tools=tools, prompt=react_prompt)
    else:
        raise ValueError(f"Unsupported model: {model_name}")
