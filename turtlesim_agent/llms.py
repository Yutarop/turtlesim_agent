import os

from langchain.agents import create_react_agent, create_tool_calling_agent
from langchain.hub import pull
from langchain_anthropic import ChatAnthropic
from langchain_cohere import ChatCohere
from langchain_google_genai import ChatGoogleGenerativeAI
from langchain_mistralai import ChatMistralAI
from langchain_ollama import ChatOllama
from langchain_openai import ChatOpenAI

from turtlesim_agent.prompts import prompt

URL = os.getenv("URL")


def create_agent(model_name: str, tools: list, temperature: float):
    """
    Create an agent with the specified language model.

    Args:
        model_name: Name of the model to use
        tools: List of tools available to the agent
        temperature: Temperature parameter for model randomness

    Returns:
        Agent instance
    """

    # OpenAI Models
    if model_name == "gpt-4o":
        llm = ChatOpenAI(model="gpt-4o", temperature=temperature)
        return create_tool_calling_agent(llm, tools=tools, prompt=prompt)
    elif model_name == "gpt-4o-mini":
        llm = ChatOpenAI(model="gpt-4o-mini", temperature=temperature)
        return create_tool_calling_agent(llm, tools=tools, prompt=prompt)
    elif model_name == "gpt-4-turbo":
        llm = ChatOpenAI(model="gpt-4-turbo", temperature=temperature)
        return create_tool_calling_agent(llm, tools=tools, prompt=prompt)
    elif model_name == "gpt-3.5-turbo":
        llm = ChatOpenAI(model="gpt-3.5-turbo", temperature=temperature)
        return create_tool_calling_agent(llm, tools=tools, prompt=prompt)

    # Anthropic Models
    elif model_name == "claude-3.5-sonnet":
        llm = ChatAnthropic(model="claude-3-5-sonnet-20241022", temperature=temperature)
        return create_tool_calling_agent(llm, tools=tools, prompt=prompt)
    elif model_name == "claude-3-opus":
        llm = ChatAnthropic(model="claude-3-opus-20240229", temperature=temperature)
        return create_tool_calling_agent(llm, tools=tools, prompt=prompt)
    elif model_name == "claude-3-haiku":
        llm = ChatAnthropic(model="claude-3-haiku-20240307", temperature=temperature)
        return create_tool_calling_agent(llm, tools=tools, prompt=prompt)
    elif model_name == "claude":  # Keep backward compatibility
        llm = ChatAnthropic(model="claude-3-opus-20240229", temperature=temperature)
        return create_tool_calling_agent(llm, tools=tools, prompt=prompt)

    # Google Models
    elif model_name == "gemini-2.0-flash":
        llm = ChatGoogleGenerativeAI(model="gemini-2.0-flash", temperature=temperature)
        return create_tool_calling_agent(llm, tools=tools, prompt=prompt)
    elif model_name == "gemini-1.5-pro":
        llm = ChatGoogleGenerativeAI(model="gemini-1.5-pro", temperature=temperature)
        return create_tool_calling_agent(llm, tools=tools, prompt=prompt)
    elif model_name == "gemini-1.5-flash":
        llm = ChatGoogleGenerativeAI(model="gemini-1.5-flash", temperature=temperature)
        return create_tool_calling_agent(llm, tools=tools, prompt=prompt)

    # Mistral AI Models
    elif model_name == "mistral-large":
        llm = ChatMistralAI(model="mistral-large-latest", temperature=temperature)
        return create_tool_calling_agent(llm, tools=tools, prompt=prompt)
    elif model_name == "mistral-medium":
        llm = ChatMistralAI(model="mistral-medium-latest", temperature=temperature)
        return create_tool_calling_agent(llm, tools=tools, prompt=prompt)
    elif model_name == "mistral-small":
        llm = ChatMistralAI(model="mistral-small-latest", temperature=temperature)
        return create_tool_calling_agent(llm, tools=tools, prompt=prompt)

    # Cohere Models
    elif model_name == "command-r-plus":
        llm = ChatCohere(model="command-r-plus", temperature=temperature)
        return create_tool_calling_agent(llm, tools=tools, prompt=prompt)
    elif model_name == "command-r":
        llm = ChatCohere(model="command-r", temperature=temperature)
        return create_tool_calling_agent(llm, tools=tools, prompt=prompt)

    # Ollama Models (Local/Self-hosted)
    elif model_name == "mistral-ollama":
        llm = ChatOllama(model="mistral", base_url=URL, temperature=temperature)
        try:
            return create_react_agent(llm, tools=tools, prompt=prompt)
        except Exception:
            react_prompt = pull("hwchase17/react")
            return create_react_agent(llm, tools=tools, prompt=react_prompt)
    elif model_name == "gemma2":
        llm = ChatOllama(model="gemma2", base_url=URL, temperature=temperature)
        try:
            return create_react_agent(llm, tools=tools, prompt=prompt)
        except Exception:
            react_prompt = pull("hwchase17/react")
            return create_react_agent(llm, tools=tools, prompt=react_prompt)
    elif model_name == "llama3.1":
        llm = ChatOllama(model="llama3.1", base_url=URL, temperature=temperature)
        try:
            return create_react_agent(llm, tools=tools, prompt=prompt)
        except Exception:
            react_prompt = pull("hwchase17/react")
            return create_react_agent(llm, tools=tools, prompt=react_prompt)
    elif model_name == "qwen2.5":
        llm = ChatOllama(model="qwen2.5", base_url=URL, temperature=temperature)
        try:
            return create_react_agent(llm, tools=tools, prompt=prompt)
        except Exception:
            react_prompt = pull("hwchase17/react")
            return create_react_agent(llm, tools=tools, prompt=react_prompt)
    elif model_name == "phi3":
        llm = ChatOllama(model="phi3", base_url=URL, temperature=temperature)
        try:
            return create_react_agent(llm, tools=tools, prompt=prompt)
        except Exception:
            react_prompt = pull("hwchase17/react")
            return create_react_agent(llm, tools=tools, prompt=react_prompt)

    # Backward compatibility for existing model names
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
        available_models = [
            # OpenAI
            "gpt-4o",
            "gpt-4o-mini",
            "gpt-4-turbo",
            "gpt-3.5-turbo",
            # Anthropic
            "claude-3.5-sonnet",
            "claude-3-opus",
            "claude-3-haiku",
            "claude",
            # Google
            "gemini-2.0-flash",
            "gemini-1.5-pro",
            "gemini-1.5-flash",
            # Mistral AI
            "mistral-large",
            "mistral-medium",
            "mistral-small",
            # Cohere
            "command-r-plus",
            "command-r",
            # Ollama
            "mistral-ollama",
            "gemma2",
            "llama3.1",
            "qwen2.5",
            "phi3",
            # Backward compatibility
            "mistral",
            "gemma3",
        ]
        raise ValueError(
            f"Unsupported model: {model_name}. Available models: {', '.join(available_models)}"
        )


def get_available_models():
    """
    Get list of all available models grouped by provider.

    Returns:
        dict: Dictionary of models grouped by provider
    """
    return {
        "OpenAI": ["gpt-4o", "gpt-4o-mini", "gpt-4-turbo", "gpt-3.5-turbo"],
        "Anthropic": ["claude-3.5-sonnet", "claude-3-opus", "claude-3-haiku", "claude"],
        "Google": ["gemini-2.0-flash", "gemini-1.5-pro", "gemini-1.5-flash"],
        "Mistral AI": ["mistral-large", "mistral-medium", "mistral-small"],
        "Cohere": ["command-r-plus", "command-r"],
        "Ollama (Local)": [
            "mistral-ollama",
            "gemma2",
            "llama3.1",
            "qwen2.5",
            "phi3",
            "mistral",
            "gemma3",
        ],
    }
