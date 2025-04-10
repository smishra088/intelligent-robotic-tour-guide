import pyaudio
import wave
import io
from google.cloud import speech
from google.oauth2 import service_account

GOOGLE_KEY_PATH = "/home/sweta/Documents/keys/google_stt_key.json"
RATE = 16000
CHUNK = 1024
FORMAT = pyaudio.paInt16
CHANNELS = 1
RECORD_SECONDS = 8

def record_audio(filename):
    print(f"🎤  Please speak...")
    audio = pyaudio.PyAudio()
    stream = audio.open(format=FORMAT, channels=CHANNELS, rate=RATE, input=True, frames_per_buffer=CHUNK)
    frames = [stream.read(CHUNK) for _ in range(0, int(RATE / CHUNK * RECORD_SECONDS))]
    stream.stop_stream()
    stream.close()
    audio.terminate()

    with wave.open(filename, 'wb') as wf:
        wf.setnchannels(CHANNELS)
        wf.setsampwidth(audio.get_sample_size(FORMAT))
        wf.setframerate(RATE)
        wf.writeframes(b''.join(frames))

def transcribe_audio(filename):
    credentials = service_account.Credentials.from_service_account_file(GOOGLE_KEY_PATH)
    client = speech.SpeechClient(credentials=credentials)
    with io.open(filename, "rb") as audio_file:
        content = audio_file.read()
    audio_data = speech.RecognitionAudio(content=content)
    config = speech.RecognitionConfig(
        encoding=speech.RecognitionConfig.AudioEncoding.LINEAR16,
        sample_rate_hertz=RATE,
        language_code="en-US"
    )
    response = client.recognize(config=config, audio=audio_data)
    return response.results[0].alternatives[0].transcript if response.results else ""

if __name__ == "__main__":
    for i in range(1, 6):
        print(f"\n--- Test {i} ---")
        filename = f"test_{i}.wav"
        record_audio(filename)
        result = transcribe_audio(filename)
        if result:
            print(f"🗣️ You said: {result}")
        else:
            print("❌ Could not understand audio.")

