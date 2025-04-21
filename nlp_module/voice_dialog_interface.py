import gradio as gr
import openai
import os
import whisper
import tempfile

# Set OpenAI API key
openai.api_key = os.getenv("OPENAI_API_KEY")

# Load Whisper model once (avoid reloading every time)
model = whisper.load_model("base")

# Whisper Speech-to-Text function
def transcribe(audio):
    print("📥 AUDIO FILE RECEIVED:", audio)
    if not audio:
        print("❌ No audio received.")
        return "No audio received."

    # Save audio to temp file if it's not already a path
    if isinstance(audio, tuple):  # Gradio may return (file_obj, sample_rate)
        audio_data, _ = audio
        with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as tmp:
            tmp.write(audio_data.read())
            filename = tmp.name
    else:
        filename = audio

    try:
        result = model.transcribe(filename)
        transcript = result.get("text", "").strip()
        print("📝 Final Transcript:", transcript)
        return transcript if transcript else "No clear speech detected."
    except Exception as e:
        print("❌ Whisper Error:", e)
        return "Error in transcription"

# Extract preferences from transcript
def extract_preferences(text):
    preferences = []
    text = text.lower()
    if "avoid" in text and "crowd" in text:
        preferences.append("Avoid Crowds")
    if "sculpture" in text:
        preferences.append("Visit Sculpture Gallery")
    if "modern art" in text:
        preferences.append("Visit Modern Art Section")
    if "painting" in text:
        preferences.append("Visit Painting Section")
    if "skip" in text:
        preferences.append("Skip Section")
    return preferences if preferences else ["No explicit preferences found."]

# Main interaction function
def voice_to_gpt(audio):
    try:
        print("🎙️ Processing audio input...")
        transcript = transcribe(audio)

        if transcript in ["No audio received.", "No clear speech detected.", "Error in transcription"]:
            return transcript, "", ""

        print("🤖 Calling GPT-4...")
        response = openai.ChatCompletion.create(
            model="gpt-4",
            messages=[
                {"role": "system", "content": "You are a helpful museum guide."},
                {"role": "user", "content": transcript},
            ],
        )
        gpt_reply = response.choices[0].message["content"]
        print("💬 GPT-4 Response:", gpt_reply)

        prefs = extract_preferences(transcript)
        print("🎯 Preferences Extracted:", prefs)

        return transcript, gpt_reply, "\n".join(prefs)

    except Exception as e:
        print("❌ ERROR in voice_to_gpt():", e)
        return "Error", "Error", f"❌ {str(e)}"

# Gradio Interface
interface = gr.Interface(
    fn=voice_to_gpt,
    inputs=gr.Audio(
        sources=["microphone"],
        type="filepath",
        label="Museum Request",
        max_length=10,
        streaming=False
    ),
    outputs=[
        gr.Textbox(label="Transcript"),
        gr.Textbox(label="Assistant Response"),
        gr.Textbox(label="Extracted Preferences")
    ],
    title="Museum Assistant",
)

# Launch
interface.launch(share=True, debug=True)
