import os
import uuid
from moviepy.editor import VideoFileClip, TextClip, CompositeVideoClip
import speech_recognition as sr
from deep_translator import GoogleTranslator
from faster_whisper import WhisperModel

def translate_video(input_video_path, output_video_path):
    """
    Process a Chinese video by:
    1. Extracting audio
    2. Transcribing Chinese speech
    3. Translating transcription to English
    4. Adding English subtitles to the video
    """
    # Extract audio from video
    video = VideoFileClip(input_video_path)
    audio_path = f"{uuid.uuid4()}_audio.wav"
    video.audio.write_audiofile(audio_path)

    # Transcribe audio using Whisper (supports multiple languages)
    model = WhisperModel("base", device="cpu", compute_type="int8")
    segments, _ = model.transcribe(audio_path, language="zh")
    
    # Prepare transcription and translation
    transcriptions = []
    for segment in segments:
        chinese_text = segment.text.strip()
        if chinese_text:
            english_text = GoogleTranslator(source='zh-CN', target='en').translate(chinese_text)
            transcriptions.append({
                'start': segment.start,
                'end': segment.end,
                'chinese': chinese_text,
                'english': english_text
            })

    # Create subtitle clips
    subtitle_clips = []
    font_size = 24
    font_color = 'white'
    stroke_color = 'black'
    
    for trans in transcriptions:
        subtitle = TextClip(
            trans['english'], 
            fontsize=font_size, 
            color=font_color, 
            stroke_color=stroke_color, 
            stroke_width=2,
            font='Arial',
            size=(video.size[0] * 0.9, None)
        ).set_position(('center', 'bottom'))
        
        subtitle = subtitle.set_start(trans['start']).set_duration(trans['end'] - trans['start'])
        subtitle_clips.append(subtitle)

    # Combine video with subtitles
    final_video = CompositeVideoClip([video] + subtitle_clips)
    final_video.write_videofile(output_video_path, codec='libx264', audio_codec='aac')

    # Clean up temporary files
    os.remove(audio_path)
    video.close()
    final_video.close()

# Example usage
input_video = 'input_video.mp4'
output_video = 'output_english_subtitled_video.mp4'
translate_video(input_video, output_video)