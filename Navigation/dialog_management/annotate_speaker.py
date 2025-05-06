from pyannote.audio import Pipeline
from transformers import pipeline as hf_pipeline
import librosa
import soundfile as sf
import os

# Set huggingface API key
HF_API_KEY = os.getnev("HF_API_KEY")

def annotate_audio(audio_file):
    speaker_dict = {}
    pipeline = Pipeline.from_pretrained("pyannote/speaker-diarization", use_auth_token=HF_API_KEY)

    diarization = pipeline(audio_file)

    asr_pipeline = hf_pipeline("automatic-speech-recognition", model="openai/whisper-base")

    # Load audio file and resample to 16kHz
    audio, rate = librosa.load(audio_file, sr=16000)

    # Process each diarization segment
    for turn, _, speaker in diarization.itertracks(yield_label=True):
        start = int(turn.start * rate)
        end = int(turn.end * rate)
        segment_audio = audio[start:end]
        
        sf.write("temp_segment.wav", segment_audio, rate)
    
        transcription = asr_pipeline("temp_segment.wav")["text"]
        print(f"{speaker} spoke from {turn.start:.1f}s to {turn.end:.1f}s: {transcription}")
        
        if speaker not in speaker_dict:
            speaker_dict[speaker] = [transcription]
        else:
            speaker_dict[speaker].append(transcription)
        return speaker_dict
    
    
if __name__ == "__main__":
    audio_file = "tutorials_assets_sample.wav"
    speaker_history = annotate_audio(audio_file)
    print(speaker_history)