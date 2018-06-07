import ros
import pyaudio
import wave
import sys


if __name__ == '__main__':

    # This is sound processing part: we create an input stream to read from microphone.
    p = pyaudio.PyAudio()
    stream = p.open(format = p.get_format_from_width(2),
            channels = 1,
            rate=16000,
            input=True,
            output=False,
            frames_per_buffer=640)

    # This is main loop: we iteratively poll audio and gui: audio data are stored in output_buffer,
    # whereas gui is checked for stop flag value (if keyPressedEvent happened, flag will be set
    # to True and break our main loop).

    i = 0
    output_buffer = ""
    while i < 100:
        data = stream.read(640)
        output_buffer += data
        i += 1
    stream.stop_stream()
    stream.close()

    # Here we output contents of output_buffer as .wav file
    output_wav = wave.open("output.wav", 'w')
    output_wav.setparams((1, 2, 16000, len(output_buffer),"NONE","not compressed"))
    output_wav.writeframesraw(output_buffer)

    p.terminate()