import openai
import os

# Load API Key from environment
openai.api_key = os.getenv("OPENAI_API_KEY")

def gpt_response(prompt):
    response = openai.ChatCompletion.create(
        model="gpt-4", 
        messages=[
            {"role": "system", "content": "You are a helpful museum guide. Answer based on visitor commands."},
            {"role": "user", "content": prompt}
        ]
    )
    return response['choices'][0]['message']['content']

if __name__ == "__main__":
    print("\n--- Museum Guide NLP Module ---")
    user_input = input("Please enter your query: ")
    print("\nProcessing your request...")
    reply = gpt_response(user_input)
    print("\nGPT-4 Response:\n")
    print(reply)


