def get_gpt_response(prompt):
    print("Simulated GPT-4 prompt:", prompt)

    prompt_lower = prompt.lower()

    if "sculpture" in prompt_lower:
        return "Sure! The sculpture gallery is down the main hall to your right."
    
    elif "modern art" in prompt_lower:
        return "Absolutely! Let's head over to the modern art exhibit."
    
    elif "history" in prompt_lower or "historical" in prompt_lower:
        return "Great choice! The history section is located on the second floor."

    elif "avoid crowd" in prompt_lower or "less crowded" in prompt_lower or "skip crowded" in prompt_lower:
        return "Understood. I’ll guide you through quieter areas of the museum."

    elif "exit" in prompt_lower:
        return "The nearest exit is through the main lobby. Would you like me to guide you there?"

    elif "where am i" in prompt_lower or "location" in prompt_lower:
        return "You’re currently in the central exhibit hall. Let me know where you’d like to go next."

    elif "what's next" in prompt_lower or "next stop" in prompt_lower:
        return "Next, we’ll be heading to the impressionist gallery. Let me know if you’d like to change that."

    elif "help" in prompt_lower or "guide" in prompt_lower:
        return "I’m here to help! You can say things like 'Take me to modern art' or 'Avoid crowded exhibits'."

    else:
        return "Got it! I’ll personalize your tour based on your interests."
    

# Test the function
if __name__ == "__main__":
    test_prompt = "Take me to the sculpture gallery"
    print("User:", test_prompt)
    print("GPT-4 (mock):", get_gpt_response(test_prompt))
