# NLP Module (GPT-4 Mock) – Intelligent Robotic Tour Guide

**Implemented by: Sweta Mishra**  

## Overview

This module is part of the Intelligent Robotic Conversational and Interaction System for a simulated museum tour guide. It is responsible for handling complex user queries and generating intelligent, context-aware responses using natural language processing techniques.

Due to quota restrictions with OpenAI’s GPT-4 API, a mock GPT-4 interface was implemented to simulate intelligent responses. This enables seamless system integration and testing while maintaining compatibility with the OpenAI SDK for future deployment.

## Key Functionalities

- **Simulated GPT-4 Interface**  
  A Python module (`gpt_interface.py`) was developed to simulate GPT-4 responses based on user prompts using predefined logic.

- **Multi-turn Capability (Pluggable)**  
  The module is structured to support multi-turn conversations, enabling future integration with live GPT-4 or similar LLMs.

- **Command Understanding**  
  Handles tour-related commands such as:
  - “Take me to the sculpture gallery”
  - “Show me modern art”

- **Personalization-Ready**  
  The module is designed to be extended with user-specific data to provide personalized responses in future versions.

## Sample Code Logic

```python
def get_gpt_response(prompt):
    if "sculpture" in prompt.lower():
        return "Sure! The sculpture gallery is down the main hall to your right."
    elif "modern art" in prompt.lower():
        return "Absolutely! Let's head over to the modern art exhibit."
    else:
        return "Great! I’ll guide you through the museum based on your interests."
