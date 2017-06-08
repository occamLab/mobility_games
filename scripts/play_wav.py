import pyaudio
import wave
import time
import sys
from pydub import AudioSegment

class Wav():
    def __init__(self, filename):
        #   Create wav object
        self.wf = wave.open(filename, 'rb')
        self.p = pyaudio.PyAudio()
        self.stream = self.p.open(format=self.p.get_format_from_width(self.wf.getsampwidth()),
                        channels=self.wf.getnchannels(),
                        rate=self.wf.getframerate(),
                        output=True,
                        stream_callback=self.callback)
        self.stream.stop_stream()

    def callback(self, in_data, frame_count, time_info, status):
        data = self.wf.readframes(frame_count)
        return (data, pyaudio.paContinue)

    def play(self):
        #   Start playing wav
        self.stream.start_stream()

    def pause(self):
        #   Pause sound
        self.stream.stop_stream()

    def close(self):
        #   Stop and close wav object
        self.stream.close()
        self.wf.close()
        self.p.terminate()
