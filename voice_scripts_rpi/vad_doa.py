
import sys
import webrtcvad
import numpy as np
from mic_array import MicArray
from pixels import Pixels, pixels
import apa102
from alexa_led_pattern import AlexaLedPattern
from google_home_led_pattern import GoogleHomeLedPattern

RATE = 16000
CHANNELS = 4
VAD_FRAMES = 10     # ms
DOA_FRAMES = 200    # ms


def main():
    vad = webrtcvad.Vad(3)

    speech_count = 0
    chunks = []
    doa_chunks = int(DOA_FRAMES / VAD_FRAMES)
    pixels.pattern = GoogleHomeLedPattern(show=pixels.show)
    dev = apa102.APA102(num_led=12)
    try:
        with MicArray(RATE, CHANNELS, RATE * VAD_FRAMES / 1000)  as mic:
            for chunk in mic.read_chunks():
                # Use single channel audio to detect voice activity
                if vad.is_speech(chunk[0::CHANNELS].tobytes(), RATE):
                    speech_count += 1

                sys.stdout.flush()
                
                chunks.append(chunk)
                if len(chunks) == doa_chunks:
                    if speech_count > (doa_chunks / 2):
                        frames = np.concatenate(chunks)
                        direction = mic.get_direction(frames)
                        #pixel_ring.set_direction(direction)
                        print('\n{}'.format(int(direction)))
                        i = int(direction/30)
                        dev.set_pixel(i, 255, 0, 0)
                        for j in range(12):
                            if not(j == i):
                                dev.set_pixel(j , 0, 0, 0)
                        dev.show()
                    else:
                        pixels.off()
                    speech_count = 0
                    chunks = []

    except KeyboardInterrupt:
        pass
        
    pixel_ring.off()


if __name__ == '__main__':
    main()
