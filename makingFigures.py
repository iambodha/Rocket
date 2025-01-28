from moviepy.editor import VideoFileClip, TextClip, CompositeVideoClip

video_file = "input_video.mp4"
clip = VideoFileClip(video_file)
cut_clip = clip.subclip(10, 12)

text = TextClip("Launcher Closeup", fontsize=50, color='white')
text = text.set_position('center').set_duration(cut_clip.duration)

final_clip = CompositeVideoClip([cut_clip, text])
gif_file = "output.gif"
final_clip.write_gif(gif_file, fps=50)

print(f"GIF saved as {gif_file}")
